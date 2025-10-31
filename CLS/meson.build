project('CLS', 'cpp',
  version: '0.1.0',
  default_options: [
    'cpp_std=c++17',
    'warning_level=2',
    'buildtype=debug'
  ]
)

# Include directories
inc_dirs = include_directories(
  'include',
  'include/Wittenstein',
  'include/mavlink/c_library_v2',
  'src'
)

# Wittenstein library sources
wittenstein_sources = files(
  'include/Wittenstein/CLSUtils.cc',
  'include/Wittenstein/WAS_Comms_Api.cc'
)

# MARSH connection sources
marsh_sources = files(
  'src/marshconnection.cc',
  'src/timer.cc'
)

# Create Wittenstein static library
wittenstein_lib = static_library('wittenstein',
  wittenstein_sources,
  include_directories: inc_dirs
)

# Create MARSH static library
marsh_lib = static_library('marsh',
  marsh_sources,
  include_directories: inc_dirs
)

# Main executable
clsstream_exe = executable('clsstream',
  'CLSstream.cc',
  include_directories: inc_dirs,
  link_with: [wittenstein_lib, marsh_lib],
  install: true
)
