from setuptools import setup

package_name = 'gdp_proxy_for_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gdpmobile1',
    maintainer_email='kych@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = gdp_proxy_for_ros.talker:main',
            'listener = gdp_proxy_for_ros.listener:main',
            'proxy = gdp_proxy_for_ros.proxy:main'
        ],
    },
)
