
.. _intro:

Introduction to kaiju
======================

`kaiju <https://pacificrim.fandom.com/wiki/Kaiju>`_ is a Python-wrapped C++ package used to compute non-colliding trajectories for SDSS-V robotic focal plane positioners during field reconfiguration.  kaiju also provides routines to optimize target to fiber allocations for the robotic focal plane array.


.. _intro-simple:

A simple kaiju program
-----------------------

.. code-block:: python

    import kaiju
    kaiju.destroy()


