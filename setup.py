from setuptools import setup, find_packages

VERSION = '0.0.2'
DESCRIPTION = 'Bike suspension kinematic solver'
LONG_DESCRIPTION = ''

setup(
    
    name = "bikinematicsolver",
    version = VERSION,
    author = "mb",
    author_email = "markbak7@gmail.com",
    description = DESCRIPTION,
    long_description = LONG_DESCRIPTION,
    packages = find_packages(),
    install_requires=['dijkstar','numpy','scipy'],

    )
