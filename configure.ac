AC_PREREQ(2.64)

AC_INIT([mvm],[1.0.0])
AM_INIT_AUTOMAKE([-Wall -Werror gnu foreign])

AC_ARG_WITH(sanitized-headers,
        AS_HELP_STRING([--with-sanitized-headers=DIR],
                [Specify the location of the sanitized Linux headers]),
        [CPPFLAGS="$CPPFLAGS -I$withval"])

AC_ARG_WITH([kernel],
        AS_HELP_STRING([--with-kernel=DIR],
                [location of a built kernel; used for building kernel modules]),
        [case "${with_kernel}" in
          yes|'') AC_MSG_ERROR([--with-kernel must specify a path]) ;;
          no)     with_kernel= ;;
          *)
                if [test ! -f ${with_kernel}/Makefile]; then
                        AC_MSG_ERROR([bad value '${with_kernel}' for --with-kernel; cannot find '${with_kernel}/Makefile'])
                fi ;;
        esac])

if [test -z "${with_kernel}"]; then
       AC_MSG_ERROR([no kernel specified; kernel modules will not be included])
else
       AC_SUBST([KERNEL_DIR], [${with_kernel}])
fi

AC_CONFIG_FILES([
    Makefile
    driver/Makefile
    ])
AC_OUTPUT
