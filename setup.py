import sys
from setuptools import setup, Extension, find_packages
from shutil import rmtree, copyfile
import glob
import os
from shutil import copyfile

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

def getCoordioInclude():
    import coordio
    d = os.path.dirname(coordio.__file__)
    return os.path.abspath(os.path.join(d, "..", "include"))

def getCoordioSrc():
    import coordio
    d = os.path.dirname(coordio.__file__)
    return os.path.abspath(os.path.join(d, "..", "cextern", "conv.cpp"))


def getIncludes():
    return [
        'include',
        # '/usr/local/include',
        # '/usr/local/include/eigen3',
        # '/usr/include/eigen3',
        # '/usr/include',
        getPybindInclude(),
        getPybindInclude(user=True),
        getCoordioInclude()
    ]


def getVersion():
    with open("python/kaiju/__version__.py", "r") as f:
        lines = f.readlines()
    for line in lines:
        l = line.strip()
        if line.startswith("__version__"):
            v = line.split("=")[-1].strip().strip('"').strip("'")
            return v


def getSources():

    return [
        'src/cKaiju.cpp',
        'src/robot.cpp',
        'src/robotGrid.cpp',
        'src/utils.cpp',
        'src/target.cpp',
        'src/fiducial.cpp',
        getCoordioSrc()
    ]

extra_compile_args = ["--std=c++11", "-fPIC", "-v", "-O3"]
extra_link_args = None
if sys.platform == 'darwin':
    extra_compile_args += ['-stdlib=libc++', '-mmacosx-version-min=10.9']
    extra_link_args = ["-v", '-mmacosx-version-min=10.9']

    # extra_compile_args += ['-mmacosx-version-min=10.9', '-stdlib=libc++']

module = Extension(
    'kaiju/cKaiju',
    include_dirs=getIncludes(),
    extra_compile_args=extra_compile_args,
    extra_link_args = extra_link_args,
    sources=getSources()
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
        # setup_requires=["sdss-coordio>0.1.0"], also add sdss-coordio>0.1.0 to requirements.txt
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


#if sys.argv[-1] == "build":
buildDir = glob.glob("build/lib*")[0]
soFile = glob.glob(buildDir + "/kaiju/cKaiju*so")[0]
base, filename = os.path.split(soFile)
dest = "python/kaiju/%s"%filename
copyfile(soFile, dest)
mode = os.stat(dest).st_mode
mode |= (mode & 0o444) >> 2
os.chmod(dest, mode)


