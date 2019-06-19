import sys
from setuptools import setup, Extension
# import pybind11
import glob
import os


class getPybindInclude(object):
    """Helper class to determine the pybind11 include path
    The purpose of this class is to postpone importing pybind11
    until it is actually installed, so that the ``get_include()``
    method can be invoked.
    https://github.com/pybind/python_example/blob/master/setup.py
    """

    def __init__(self, user=False):
        self.user = user

    def __str__(self):
        import pybind11
        return pybind11.get_include(self.user)

def getIncludes():
    return [
        'include',
        '/usr/local/include',
        '/usr/local/include/eigen3',
        '/usr/include',
        getPybindInclude(),
        getPybindInclude(user=True)
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
    include_dirs=getIncludes(),
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
    install_requires=[line.strip() for line in open("requirements.txt")]
)

# put the shared object in a standard location
# not
buildDir = glob.glob("build/lib*")[0]
os.rename(buildDir, "build/lib")
