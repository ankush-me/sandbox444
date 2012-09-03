# Install script for directory: /home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/sibi/sandbox/sandbox444/filter_cloud_color/lib/liblog4cplus.a")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/log4cplus" TYPE FILE FILES
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/appender.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/asyncappender.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/clogger.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/config.hxx"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/configurator.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/consoleappender.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/fileappender.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/fstreams.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/hierarchy.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/hierarchylocker.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/layout.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/logger.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/loggingmacros.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/loglevel.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/mdc.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/ndc.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/nteventlogappender.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/nullappender.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/socketappender.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/streams.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/syslogappender.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/tchar.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/tracelogger.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/tstring.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/version.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/win32debugappender.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/win32consoleappender.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/log4cplus/boost" TYPE FILE FILES "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/boost/deviceappender.hxx")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/log4cplus/helpers" TYPE FILE FILES
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/appenderattachableimpl.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/fileinfo.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/lockfile.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/loglog.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/logloguser.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/pointer.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/property.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/queue.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/sleep.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/snprintf.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/socket.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/socketbuffer.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/stringhelper.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/thread-config.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/helpers/timehelper.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/log4cplus/internal" TYPE FILE FILES
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/internal/env.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/internal/internal.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/internal/socket.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/log4cplus/spi" TYPE FILE FILES
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/spi/appenderattachable.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/spi/factory.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/spi/filter.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/spi/loggerfactory.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/spi/loggerimpl.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/spi/loggingevent.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/spi/objectregistry.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/spi/rootlogger.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/log4cplus/thread/impl" TYPE FILE FILES
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/thread/impl/syncprims-impl.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/thread/impl/syncprims-pthreads.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/thread/impl/syncprims-win32.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/thread/impl/threads-impl.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/thread/impl/tls.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/log4cplus/thread" TYPE FILE FILES
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/thread/syncprims-pub-impl.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/thread/syncprims.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/thread/threads.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/log4cplus/config" TYPE FILE FILES
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/config/macosx.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/config/win32.h"
    "/home/sibi/sandbox/sandbox444/lib/log4cplus-1.1.0-rc3/src/../include/log4cplus/config/windowsh-inc.h"
    "/home/sibi/sandbox/sandbox444/filter_cloud_color/include/log4cplus/config/defines.hxx"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

