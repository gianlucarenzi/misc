/************************************************
 * $Id$
 ***********************************************/

#ifndef EC_TYPES_H_INCLUDED
#define EC_TYPES_H_INCLUDED

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef __GNUC__
#error "Supportato solo GNU GCC"
#endif


//Questa e` utile anche sul PC
#ifndef PACKED
# define PACKED	__attribute__ ((packed))
#endif

#ifndef WEAK
# define WEAK __attribute__ ((weak))
#endif

#ifndef ALIAS
# define ALIAS(f) __attribute__ ((weak, alias (#f)))
#endif

//Queste servono solo sull'embedded
#if defined(__GNUC__) && defined(GCC_ARMCM3)
# define EXTFLASH __attribute__ ((section(".external_flash")))
# define EXTSRAM __attribute__ ((section(".external_sram")))
//# define EXTSRAM __attribute__ ((section(".external_sram"))) __attribute__ ((nocommon))
# define APPFLASH __attribute__ ((section(".application_flash")))
# define NORETURN_ATTRIBUTE __attribute__ ((noreturn))
#else
# define EXTFLASH
# define EXTSRAM
# define APPFLASH
# define NORETURN_ATTRIBUTE
#endif

#ifndef true
# define true	1
# define false	0
# warning "Defining true and false"
#endif

#ifndef HIDDEN
# define HIDDEN			static
#endif
#ifndef HIDDEN_INLINE
# define HIDDEN_INLINE	static inline
#endif

// Force a compilation error if condition is true
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2 * !!(condition)]))

/* Force a compilation error if condition is true, but also produce a
   result (of value 0 and type size_t), so the expression can be used
   e.g. in a structure initializer (or where-ever else comma expressions
   aren't permitted). */
#define BUILD_BUG_ON_ZERO(e) (sizeof(char[1 - 2 * !!(e)]) - 1)

#ifndef __cplusplus
	#ifdef __GNUC__
		// &a[0] degrades to a pointer: a different type from an array
		#define __must_be_array(a) \
			BUILD_BUG_ON_ZERO(__builtin_types_compatible_p(typeof(a), typeof(&a[0])))
	#else
		#define __must_be_array(a) 0
	#endif

	#define ArraySize(arr)	(sizeof(arr) / sizeof((arr)[0]) + __must_be_array(arr))
#else
	#if 0	// only valid if compiled with -std=c++0x
		template <typename T, size_t N>
		constexpr size_t _array_size(T (&)[N]) { return N; }
		#define ArraySize(arr)	_array_size(arr)
	#else
		//Tricky but valid even in old C++
		// public interface
		#define ArraySize(x)	(												\
			0 * sizeof( reinterpret_cast<const ::Bad_arg_to_COUNTOF*>(x) ) +	\
			0 * sizeof( ::Bad_arg_to_COUNTOF::check_type((x), &(x)) ) +			\
			sizeof(x) / sizeof((x)[0])											\
		)

		// implementation details
		struct Bad_arg_to_COUNTOF
		{
			class Is_pointer;	// intentionally incomplete type
			class Is_array {};
			template <typename T>
			static Is_pointer check_type(const T*, const T* const*);
			static Is_array check_type(const void*, const void*);
		};
	#endif
#endif

#define KiB(x)	((x) * 1024L)
#define MiB(x)	(KiB(x) * 1024L)

//#ifndef max
//#define max(a,b)		( (a) > (b) ? (a) : (b) )
//#endif

//#ifndef min
//#define min(a,b)		( (a) < (b) ? (a) : (b) )
//#endif

#ifndef assert_param
# ifdef  USE_FULL_ASSERT
#  define assert_param(expr) ((expr) ? (void)0 : ec_assert_failed(__FILE__, __LINE__, __FUNCTION__, #expr))
   extern void ec_assert_failed(const char *file, unsigned int line, const char *func, const char *expr);
# else
#  define assert_param(expr)
# endif
#endif

/**
 * ContainerOf - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define ContainerOf(ptr, type, member) ({ \
	const typeof( ((type *)0)->member ) *__mptr = (ptr); \
	(type *)( (char *)__mptr - offsetof(type,member) );})

#define FieldSizeOf(t, f) (sizeof(((t *)0)->f))

enum {
	OK = 0,
#define OK OK
	ECERR_IO,
#define ECERR_IO ECERR_IO
	ECERR_OUTOFMEM,
#define ECERR_OUTOFMEM ECERR_OUTOFMEM
	ECERR_BADPARAM,
#define ECERR_BADPARAM ECERR_BADPARAM
	ECERR_IGNORE_PACKET = 127,
#define ECERR_IGNORE_PACKET ECERR_IGNORE_PACKET
	ECERR_MAGIC_SIGNATURE_PACKET_PROGRAMMING = 7001,
#define ECERR_MAGIC_SIGNATURE_PACKET_PROGRAMMING ECERR_MAGIC_SIGNATURE_PACKET_PROGRAMMING
	ECERR_MAGIC_SIGNATURE_PACKET_FIRMWARE_UPLOAD = 7000,
#define ECERR_MAGIC_SIGNATURE_PACKET_FIRMWARE_UPLOAD ECERR_MAGIC_SIGNATURE_PACKET_FIRMWARE_UPLOAD
};

#define contains(__s1, __s2)    strncmp(__s1, __s2, strlen(__s2))

#define MULTICAST_PORT    6500

#endif
