import os
import sys
import urllib.request

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext as _build_ext


COORDIO_BASE = 'https://raw.githubusercontent.com/sdss/coordio/master/'

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
        'python/kaiju/include',
        getPybindInclude(),
        getPybindInclude(user=True),
    ]


def getVersion():
    with open("python/kaiju/__version__.py", "r") as f:
        lines = f.readlines()
    for line in lines:
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
        'src/gfa.cpp',
        'src/conv.cpp'
    ]


extra_compile_args = ["--std=c++11", "-fPIC", "-v", "-O3"]
extra_link_args = []
if sys.platform == 'darwin':
    extra_compile_args += ['-stdlib=libc++', '-mmacosx-version-min=10.9']
    extra_link_args = ["-v", '-mmacosx-version-min=10.9', '-L.']

    from distutils import sysconfig
    vars = sysconfig.get_config_vars()
    vars['LDSHARED'] = vars['LDSHARED'].replace('-bundle', '-dynamiclib')

extensions = [
    Extension(
        'kaiju.cKaiju',
        include_dirs=getIncludes(),
        extra_compile_args=extra_compile_args,
        extra_link_args=extra_link_args,
        sources=getSources())
]


# cKaiju relies on some funcitons defined in libcoordio in coordio. Since linking against
# that library directly is painful, we just download the latest files from GitHub
# and add them to cKaiju ...
class build_ext(_build_ext):

    def run(self):

        DIRNAME = os.path.dirname(__file__)

        with urllib.request.urlopen(COORDIO_BASE + 'src/coordio/include/coordio.h') as r:
            data = r.read()
            with open(os.path.join(DIRNAME, 'python/kaiju/include/coordio.h'), 'wb') as f:
                f.write(data)

        with urllib.request.urlopen(COORDIO_BASE + 'cextern/conv.cpp') as r:
            data = r.read()
            with open(os.path.join(DIRNAME, 'src/conv.cpp'), 'wb') as f:
                f.write(data)

        super().run()


def runSetup(packages, requirements):
    setup(
        name="sdss-kaiju",
        version=getVersion(),
        license="BSD3",
        author="Conor Sayres",
        author_email="csayres@uw.edu",
        description="Collision Avoidance for SDSS-V Positioners",
        long_description=open('README.rst').read(),
        packages=packages,
        cmdclass={'build_ext': build_ext},
        package_dir={'': 'python'},
        package_data={'kaiju': ['python/kaiju/include/*']},
        include_package_data=True,
        url="https://github.com/sdss/kaiju",
        keywords="astronomy software",
        ext_modules=extensions,
        install_requires=requirements,
        setup_requires=[
            "sdss-coordio>=1.2.1",
            "pybind11>=2.2.4,<3"
        ],
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
