from setuptools import setup, find_packages

package_name = 'unilabos'

setup(
    name=package_name,
    version='0.9.7',
    packages=find_packages(),
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junhan Chang',
    maintainer_email='changjh@pku.edu.cn',
    description='',
    license='GPL v3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "unilab = unilabos.app.main:main",
            "unilab-register = unilabos.app.register:main"
        ],
    },
)
