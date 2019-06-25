import sys
from setuptools import setup, Extension, find_packages
from shutil import rmtree
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
        # '/usr/local/include',
        '/usr/local/include/eigen3',
        '/usr/include/eigen3',
        # '/usr/include',
        getPybindInclude(),
        getPybindInclude(user=True)
    ]


def getVersion():
    with open("python/kaiju/__version__.py", "r") as f:
        lines = f.readlines()
    for line in lines:
        l = line.strip()
        if line.startswith("__version__"):
            v = line.split("=")[-1].strip().strip('"').strip("'")
            return v


sources = [
    'src/cKaiju.cpp',
    'src/robot.cpp',
    'src/robotGrid.cpp',
    'src/utils.cpp',
    'src/betaArm.cpp'
]

extra_compile_args = ["--std=c++11", "-fPIC", "-v", "-O3"]
if sys.platform == 'darwin':
    extra_compile_args += ['-mmacosx-version-min=10.9']#, '-stdlib=libc++']

module = Extension(
    'python/kaiju/cKaiju',
    include_dirs=getIncludes(),
    extra_compile_args=extra_compile_args,
    sources=sources
)

def runSetup(packages, requirements):
    setup(
        name="sdss-kaiju",
        version=getVersion(),
        license="BSD3",
        author="Conor Sayres",
        author_email="csayres@uw.edu",
        description="Collision Avoidance for SDSS-V Positioners",
        long_description=open('README.rst').read(),
        packages=packages, #["python/kaiju"],
        package_dir={'': 'python'},
        url="https://github.com/sdss/kaiju",
        keywords="astronomy software",
        ext_modules=[module],
        install_requires=requirements,
        classifiers=[
            'Intended Audience :: Science/Research',
            'License :: OSI Approved :: BSD License',
            'Natural Language :: English',
            'Operating System :: OS Independent',
            'Programming Language :: Python',
        ]
    )

requirementsFile = os.path.join(os.path.dirname(__file__), "requirements.txt")
requirements = [line.strip() for line in open(requirementsFile)]
packages = find_packages(where="python")
print("found packages", packages)
runSetup(packages, requirements)

if sys.argv[-1] == "build":
    target = "build/lib"
    if os.path.exists(target):
        print("removing target")
        rmtree(target)
    # clean up the old lib directory if present
    # put the shared object in a standard location
    buildDir = glob.glob("build/lib*")[0]
    os.rename(buildDir, target)
