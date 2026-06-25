
#ifndef TOPPRA_EXPORT_H
#define TOPPRA_EXPORT_H

#ifdef TOPPRA_STATIC_DEFINE
#  define TOPPRA_EXPORT
#  define TOPPRA_NO_EXPORT
#else
#  ifndef TOPPRA_EXPORT
#    ifdef toppra_EXPORTS
        /* We are building this library */
#      define TOPPRA_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define TOPPRA_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef TOPPRA_NO_EXPORT
#    define TOPPRA_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef TOPPRA_DEPRECATED
#  define TOPPRA_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef TOPPRA_DEPRECATED_EXPORT
#  define TOPPRA_DEPRECATED_EXPORT TOPPRA_EXPORT TOPPRA_DEPRECATED
#endif

#ifndef TOPPRA_DEPRECATED_NO_EXPORT
#  define TOPPRA_DEPRECATED_NO_EXPORT TOPPRA_NO_EXPORT TOPPRA_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef TOPPRA_NO_DEPRECATED
#    define TOPPRA_NO_DEPRECATED
#  endif
#endif

#endif /* TOPPRA_EXPORT_H */
