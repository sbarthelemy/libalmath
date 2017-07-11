Overview
========

The objective of this project is to

 1. import a SBR robot model from its urdf description
 2. import/export qi animations (.qianim files)
 3. perform some motion analysis (balance, collision) on the animation
    and show the result directly in the Maya GUI
 4. control a connected robot state (life, wake up/rest) directly from
    the Maya GUI
 5. play a qi animation on a connected robot directly from the Maya GUI


Design
======

Maya supports extensions written in C++, Python and MEL.

Maya SDK for Windows only supports 64 bits for both python and C++.

This plugin uses a mix of C++ and python.


URDF and qi animation import/export
-----------------------------------

For URDF import and .qianim import and export, C++ is used, since it enables
us to leverage the utilities from the almath library.

In order to lower toolchain requirements and avoid packaging issues,
the dependencies are kept to a minimum and no DLL is used.

in particular:

 * almath is compiled with minimalistic options
 * almath is compiled as a static library
 * only header-only parts of the boost library are used

For testing, parts of almathinternal are needed too.
Those are compiled as part of the test binary to avoid pulling the whole
almathinternal library and its dependencies.
This is hackish and fragile though.


GUI
---

We use the python Qt bindings, which are included in the Maya SDK.


Robot communication
-------------------

libqi is needed for communication with the robot.

However, we currently don't have a 64 bits SDK on Windows
(neither in C++ nor python) and compiling one would be a lot of work.

The workaround is to let the Maya 64 bits python plugin call 32 bits
binaries using (the subprocess module) for robot communication.

This works but is probably inefficient, and prevents us to keep a persistent
connection to the robot (and thus prevents us to keep the robot life focus).

How to build
============

prepare a qibuild worktree with just:

| agility
| lib/libalmath
| sdk/libgtest

Then::

  cd agility/mayaplugin
  qibuild configure -DALMATH_WITH_QIGEOMETRY=OFF -DWITH_PYTHON=OFF -DALMATH_AS_STATIC_LIBRARY=ON --release
  qibuild make
  qitest run

prepare a end-user Maya module::

  rm -rf /tmp/sbrmp
  qibuild install /tmp/sbrmp --no-packages --runtime

Troubleshooting
---------------

 * if configuring gtest fails with::

    FindPackageHandleStandardArgs.cmake:137 (message):

          Could NOT find PTHREAD (missing: PTHREAD_LIBRARIES)

  It is probably because you are compiling in 32 bits instead of 64 bits. You should run
  ``qibuild configure --wizard`` again and select a 64 bits compiler.

Todo
====

 * support hands animation, with mimic tags
 * Fix the pepper hands collada meshes orientation (https://redmine.aldebaran.lan/issues/37754)
 * support importQiAnim undo
 * implement/expose importQiAnim as a MPxFileTranslator
