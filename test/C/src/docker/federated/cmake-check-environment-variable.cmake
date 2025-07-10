# fail if the environment variable FOO is not set
if(DEFINED ENV{FOO})
  message("FOO is set to $ENV{FOO}")
else()
  message(FATAL_ERROR "FOO is not set")
endif()
