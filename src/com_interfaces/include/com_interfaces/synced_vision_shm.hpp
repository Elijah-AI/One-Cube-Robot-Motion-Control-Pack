/*******************************************************************************
 * synced_vision_shm.hpp
 *
 * SyncedRobotVision 共享内存传输（替代 ROS 大话题 /fusion/synced_robot_vision）。
 *
 * 背景：单帧 SyncedRobotVision 约 19.5MB（RGB Image ~3MB + PointCloud2 ~16MB），
 * 经 DDS 传输时一旦出现慢速远程订阅者（如 100Mbps eth1 对端），异步发送积压
 * 会把本机订阅者也拖到秒级延迟。本文件实现进程间 POSIX 共享内存环形缓冲：
 *
 *   * 发布方（berxel_camera_driver）：fusion_cb 将整帧 CDR 序列化后写入
 *     /dev/shm/berxel_fusion_shm 的一个 slot，并推进全局 publish_seq。
 *   * 订阅方（detection_pose_cloud_server / calibration_node 等）：1ms 轮询
 *     publish_seq，发现新帧后从对应 slot 拷出并用 rclcpp::Serialization
 *     反序列化。多读者互不干扰（纯读）。
 *
 * 本头文件只做"字节级"布局与读写原语，不含 rclcpp 依赖（com_interfaces 是
 * 纯消息包），(反)序列化由使用方各自完成：
 *   写侧: rclcpp::Serialization<SyncedRobotVision>::serialize_message(
 *            msg, serialized_msg) 后把 buffer 交给 Writer::publish()。
 *   读侧: Reader::read_frame() 拷出字节后调用
 *         rclcpp::Serialization<SyncedRobotVision>::deserialize_message()。
 *
 * 内存布局（总计 64B 头 + 4 × (64B slot 头 + 24MB payload) ≈ 96MB）：
 *
 *   [ MainHeader 64B ][ slot0: 64B + payload ][ slot1 ] [ slot2 ] [ slot3 ]
 *
 * 一致性协议（单写者多读者，seqlock 风格）：
 *   1. 写者按环形顺序写 slot：先 memcpy payload，再写 slot 头（seq 最后写，
 *      release 语义），最后推进主头部的 publish_seq（release 语义）。
 *   2. 读者轮询 publish_seq（acquire），seq==s 的帧位于 slot (s-1)%N。
 *   3. 读者拷出 payload 后复检：slot.seq 仍为 s 且 publish_seq < s+N
 *      （该 slot 尚未被新一轮写入复用），否则判为撕裂帧并丢弃。
 ******************************************************************************/
#ifndef COM_INTERFACES__SYNCED_VISION_SHM_HPP_
#define COM_INTERFACES__SYNCED_VISION_SHM_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace com_interfaces
{
namespace shm
{

// ---------------------------------------------------------------------------
// 常量与布局
// ---------------------------------------------------------------------------
constexpr const char * kShmName = "/berxel_fusion_shm";  // 实际位于 /dev/shm/
constexpr uint32_t kShmMagic = 0x42465348u;              // "BFSH"
constexpr uint32_t kShmVersion = 1u;
constexpr uint32_t kShmNumSlots = 4u;                    // 环形缓冲槽数
constexpr size_t kShmSlotPayload = 24ull * 1024 * 1024;   // 24MB > 单帧 CDR ~19.5MB
constexpr size_t kShmMainHeaderSize = 64;                 // 主头部占用（含填充）
constexpr size_t kShmSlotHeaderSize = 64;                 // 每槽头部占用（含填充）
constexpr size_t kShmSlotStride = kShmSlotHeaderSize + kShmSlotPayload;
constexpr size_t kShmTotalSize = kShmMainHeaderSize + kShmNumSlots * kShmSlotStride;

/// 共享内存主头部（位于映射区起始）
struct MainHeader
{
  uint32_t magic;
  uint32_t version;
  uint32_t num_slots;
  uint32_t slot_payload;
  uint64_t publish_seq;  // 最新完成写入的全局序号；0 = 尚无帧（release 写 / acquire 读）
  uint32_t reserved[10]; // 填充至 kShmMainHeaderSize
};
static_assert(sizeof(MainHeader) == kShmMainHeaderSize, "MainHeader size mismatch");

/// 每槽头部（位于该槽 payload 正前方）
struct SlotHeader
{
  uint64_t seq;         // 本槽当前帧的全局序号（写 payload 之后再 release 写入）
  uint64_t stamp_sec;    // 帧时间戳（秒），仅诊断用途
  uint64_t stamp_nsec;   // 帧时间戳（纳秒）
  uint32_t camera_id;    // 来源相机（0=头部, 1=腰部, 2+ 预留）
  uint32_t payload_size; // CDR 序列化字节数
  uint64_t reserved0;
  uint32_t pad[6];      // 填充至 kShmSlotHeaderSize
};
static_assert(sizeof(SlotHeader) == kShmSlotHeaderSize, "SlotHeader size mismatch");

inline MainHeader * main_header(uint8_t * base)
{
  return reinterpret_cast<MainHeader *>(base);
}
inline const MainHeader * main_header(const uint8_t * base)
{
  return reinterpret_cast<const MainHeader *>(base);
}
inline SlotHeader * slot_header(uint8_t * base, uint32_t idx)
{
  return reinterpret_cast<SlotHeader *>(base + kShmMainHeaderSize + idx * kShmSlotStride);
}
inline const SlotHeader * slot_header(const uint8_t * base, uint32_t idx)
{
  return reinterpret_cast<const SlotHeader *>(
    base + kShmMainHeaderSize + idx * kShmSlotStride);
}
inline uint8_t * slot_payload(uint8_t * base, uint32_t idx)
{
  return base + kShmMainHeaderSize + idx * kShmSlotStride + kShmSlotHeaderSize;
}
inline const uint8_t * slot_payload(const uint8_t * base, uint32_t idx)
{
  return base + kShmMainHeaderSize + idx * kShmSlotStride + kShmSlotHeaderSize;
}

inline void atomic_store_u64(uint64_t * p, uint64_t v)  // NOLINT
{
  __atomic_store_n(p, v, __ATOMIC_RELEASE);
}
inline uint64_t atomic_load_u64(const uint64_t * p)  // NOLINT
{
  return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

// ---------------------------------------------------------------------------
// 发布方（单写者；进程内持有一个实例即可）
// ---------------------------------------------------------------------------
class SyncedVisionShmWriter
{
public:
  SyncedVisionShmWriter() = default;
  SyncedVisionShmWriter(const SyncedVisionShmWriter &) = delete;
  SyncedVisionShmWriter & operator=(const SyncedVisionShmWriter &) = delete;
  ~SyncedVisionShmWriter() { close(); }

  /// 创建（或复用）共享内存并复位序号。失败时 err 给出原因。
  bool open(std::string * err = nullptr)
  {
    if (base_ != nullptr) {return true;}
    const int fd = ::shm_open(kShmName, O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
      if (err) {*err = std::string("shm_open(O_CREAT) failed: ") + std::strerror(errno);}
      return false;
    }
    struct stat st{};
    if (::fstat(fd, &st) != 0) {
      if (err) {*err = std::string("fstat failed: ") + std::strerror(errno);}
      ::close(fd);
      return false;
    }
    // 尺寸不符（首次创建/布局变更）：截断到目标尺寸
    if (static_cast<size_t>(st.st_size) != kShmTotalSize) {
      if (::ftruncate(fd, static_cast<off_t>(kShmTotalSize)) != 0) {
        if (err) {*err = std::string("ftruncate failed: ") + std::strerror(errno);}
        ::close(fd);
        return false;
      }
    }
    void * base = ::mmap(nullptr, kShmTotalSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) {
      if (err) {*err = std::string("mmap failed: ") + std::strerror(errno);}
      ::close(fd);
      return false;
    }
    ::close(fd);  // mmap 后即可关闭 fd
    fd_ = -1;
    base_ = static_cast<uint8_t *>(base);

    MainHeader * h = main_header(base_);
    // 布局变化（magic/version/尺寸不符）时清零重建；正常重启仅复位序号
    if (h->magic != kShmMagic || h->version != kShmVersion ||
      h->num_slots != kShmNumSlots || h->slot_payload != kShmSlotPayload)
    {
      std::memset(base_, 0, kShmTotalSize);
      h->magic = kShmMagic;
      h->version = kShmVersion;
      h->num_slots = kShmNumSlots;
      h->slot_payload = static_cast<uint32_t>(kShmSlotPayload);
    }
    // 发布方启动：序号归零（旧帧自然作废，读者以 publish_seq 为准）
    atomic_store_u64(&h->publish_seq, 0);
    next_seq_ = 0;
    return true;
  }

  void close()
  {
    if (base_ != nullptr) {
      ::munmap(base_, kShmTotalSize);
      base_ = nullptr;
    }
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const { return base_ != nullptr; }

  /**
   * 写入一帧序列化数据（线程不安全，仅由发布方数据路径单线程调用）。
   * payload 需 <= kShmSlotPayload；stamp 字段仅诊断用途。
   */
  bool publish(
    const uint8_t * payload, size_t size, uint32_t camera_id,
    uint64_t stamp_sec, uint64_t stamp_nsec, std::string * err = nullptr)
  {
    if (base_ == nullptr) {
      if (err) {*err = "shm not open";}
      return false;
    }
    if (size > kShmSlotPayload) {
      if (err) {*err = "serialized frame (" + std::to_string(size) +
        " bytes) exceeds slot capacity (" + std::to_string(kShmSlotPayload) + " bytes)";}
      return false;
    }
    const uint64_t seq = next_seq_ + 1;
    const uint32_t idx = static_cast<uint32_t>((seq - 1) % kShmNumSlots);

    // 1) 写 payload  2) 写槽头（seq 最后）  3) 推进 publish_seq
    std::memcpy(slot_payload(base_, idx), payload, size);
    SlotHeader * sh = slot_header(base_, idx);
    sh->stamp_sec = stamp_sec;
    sh->stamp_nsec = stamp_nsec;
    sh->camera_id = camera_id;
    sh->payload_size = static_cast<uint32_t>(size);
    sh->reserved0 = 0;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    atomic_store_u64(&sh->seq, seq);
    atomic_store_u64(&main_header(base_)->publish_seq, seq);
    next_seq_ = seq;
    return true;
  }

  /// 最近一次成功写入的序号（0 = 尚无帧）
  uint64_t last_seq() const { return next_seq_; }

private:
  int fd_{-1};
  uint8_t * base_{nullptr};
  uint64_t next_seq_{0};
};

// ---------------------------------------------------------------------------
// 订阅方（多读者安全；每个读者持有一个实例 + 各自的轮询线程）
// ---------------------------------------------------------------------------
class SyncedVisionShmReader
{
public:
  SyncedVisionShmReader() = default;
  SyncedVisionShmReader(const SyncedVisionShmReader &) = delete;
  SyncedVisionShmReader & operator=(const SyncedVisionShmReader &) = delete;
  ~SyncedVisionShmReader() { detach(); }

  bool is_attached() const { return base_ != nullptr; }

  /// 非阻塞 attach：共享内存尚未创建（驱动未启动）或头部未就绪时返回 false。
  /// 失败原因写入 err（可为 nullptr）。建议调用方以 50~100ms 周期重试。
  bool try_attach(std::string * err = nullptr)
  {
    if (base_ != nullptr) {return true;}
    const int fd = ::shm_open(kShmName, O_RDWR, 0666);
    if (fd < 0) {
      if (err) {*err = std::string("shm_open failed: ") + std::strerror(errno) +
        " (发布方未启动?)";}
      return false;
    }
    struct stat st{};
    if (::fstat(fd, &st) != 0 || static_cast<size_t>(st.st_size) != kShmTotalSize) {
      if (err) {*err = "shm size mismatch (等待发布方初始化完成)";}
      ::close(fd);
      return false;
    }
    void * base = ::mmap(nullptr, kShmTotalSize, PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (base == MAP_FAILED) {
      if (err) {*err = std::string("mmap failed: ") + std::strerror(errno);}
      return false;
    }
    const MainHeader * h = main_header(static_cast<const uint8_t *>(base));
    if (h->magic != kShmMagic || h->version != kShmVersion ||
      h->num_slots != kShmNumSlots || h->slot_payload != kShmSlotPayload)
    {
      if (err) {*err = "shm header mismatch (发布方初始化中?)";}
      ::munmap(base, kShmTotalSize);
      return false;
    }
    base_ = static_cast<uint8_t *>(base);
    return true;
  }

  void detach()
  {
    if (base_ != nullptr) {
      ::munmap(base_, kShmTotalSize);
      base_ = nullptr;
    }
  }

  /// 最新已发布序号；0 = 未 attach 或尚无帧
  uint64_t latest_seq() const
  {
    if (base_ == nullptr) {return 0;}
    return atomic_load_u64(&main_header(base_)->publish_seq);
  }

  /**
   * 拷出 seq 对应帧的序列化字节（seq==s 的帧位于 slot (s-1)%N）。
   * buf_capacity 建议 == kShmSlotPayload。撕裂/被覆盖帧返回 false。
   */
  bool read_frame(
    uint64_t seq, uint8_t * buf, size_t buf_capacity, size_t * out_size,
    uint32_t * camera_id, uint64_t * stamp_sec, uint64_t * stamp_nsec,
    std::string * err = nullptr) const
  {
    if (base_ == nullptr || seq == 0) {
      if (err) {*err = "not attached";}
      return false;
    }
    const uint32_t idx = static_cast<uint32_t>((seq - 1) % kShmNumSlots);
    const SlotHeader * sh = slot_header(base_, idx);
    if (atomic_load_u64(&sh->seq) != seq) {
      if (err) {*err = "slot seq mismatch (帧已被覆盖)";}
      return false;
    }
    const size_t size = sh->payload_size;
    if (size > kShmSlotPayload || size > buf_capacity) {
      if (err) {*err = "invalid payload size (" + std::to_string(size) + ")";}
      return false;
    }
    std::memcpy(buf, slot_payload(base_, idx), size);
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    // 撕裂复检：slot 头序号未变，且该槽尚未被第 N 轮写入复用
    if (atomic_load_u64(&sh->seq) != seq ||
      latest_seq() >= seq + kShmNumSlots)
    {
      if (err) {*err = "torn frame (写入与读取竞争, 丢弃)";}
      return false;
    }
    if (out_size) {*out_size = size;}
    if (camera_id) {*camera_id = sh->camera_id;}
    if (stamp_sec) {*stamp_sec = sh->stamp_sec;}
    if (stamp_nsec) {*stamp_nsec = sh->stamp_nsec;}
    return true;
  }

private:
  uint8_t * base_{nullptr};
};

}  // namespace shm
}  // namespace com_interfaces

#endif  // COM_INTERFACES__SYNCED_VISION_SHM_HPP_
