===============================================================
Robotic Assembled Timber Structures with Integral Timber Joints
===============================================================

.. This README.rst serves only as the entry point for people visiting the GitHub Repro.
.. The actual readme file index page is index.rst

.. start-badges

.. image:: https://img.shields.io/badge/License-MIT-blue.svg
    :target: https://github.com/gramaziokohler/integral_timber_joints/blob/master/LICENSE
    :alt: License MIT

.. image:: https://travis-ci.com/gramaziokohler/integral_timber_joints.svg?token=DC9xyXTmKZNpvuHxcvcN&branch=master
    :target: https://travis-ci.com/gramaziokohler/integral_timber_joints
    :alt: Travis CI

.. end-badges

.. Write project description

**Research project to investigate robotic assembly methods for timber structure with integral timber joints.**

This repository contains multiple python packages and C++ projects (among many other things).
The packages and projects are located in the `src` and `src_cpp` folders respectively.

**The list of packages and projects can be found in the main documentation here:**
`Documentation Index <https://docs.gramaziokohler.arch.ethz.ch/integral_timber_joints/>`_

Contributing
------------

Make sure you setup your local development environment correctly:

* Clone the `integral_timber_joints <https://github.com/gramaziokohler/integral_timber_joints>`_ repository.
* Install development dependencies and make the project accessible from Rhino:

::

    pip install -r requirements-dev.txt
    invoke add-to-rhino

**You're ready to start working!**

During development, use tasks on the
command line to ease recurring operations:

* ``invoke clean``: Clean all generated artifacts.
* ``invoke check``: Run various code and documentation style checks.
* ``invoke docs``: Generate documentation.
* ``invoke test``: Run all tests and checks in one swift command.
* ``invoke add-to-rhino``: Make the project accessible from Rhino.
* ``invoke``: Show available tasks.

For more details, check the `Contributor's Guide <CONTRIBUTING.rst>`_.


Credits
-------------

This repository was created by Pok Yin Victor Leung <leung@arch.ethz.ch> `@yck011522 <https://github.com/yck011522>`_ at `@gramaziokohler <https://github.com/gramaziokohler>`_
