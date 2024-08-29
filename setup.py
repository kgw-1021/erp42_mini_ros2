from setuptools import setup

package_name = 'erp-42_mini_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Geunwoo Kim',
    maintainer_email='dagonkw@gmail.com',
    description='',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ErpSerialHandler = scripts.ErpSerialHandler:main',
            'ByteHandler = scripts.ByteHandler:main',
            'ErpTeleop = scripts.ErpTeleop:main',
        ],
    },
)