import sys
from setuptools import setup, Extension
import pybind11
import glob

include_dirs = [
    'include',
    '/usr/local/include',
    '/usr/include',
    pybind11.get_include(),
    pybind11.get_include(True)
]

sources = [
    'src/cKaiju.cpp',
    'src/robot.cpp',
    'src/robotGrid.cpp',
    'src/utils.cpp',
    'src/betaArm.cpp'
]


extra_compile_args = ["--std=c++11", "-fPIC", "-v", "-O3"]
if sys.platform == 'darwin':
    extra_compile_args += ['-stdlib=libc++', '-mmacosx-version-min=10.9']

module = Extension('python/kaiju/cKaiju',
    include_dirs=include_dirs,
    extra_compile_args=extra_compile_args,
    extra_link_args = ["-v", '-mmacosx-version-min=10.9'],
    sources=sources
)

setup(
    name="kaiju",
    version="0.0.1",
    author="Conor Sayres",
    description="Collision Avoidance for SDSS-V Positioners",
    packages=["python/kaiju"],
    ext_modules=[module],
)

