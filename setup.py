from setuptools import find_packages, setup

package_name = 'py_srvcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/ORAgentClientInit.py']),
        ('lib/' + package_name, [package_name+'/ORAgentClient.py']),
        ('lib/' + package_name, [package_name+'/ClientToolkit.py']),
        ('lib/' + package_name, [package_name+'/ConvertDataFormat.py']),
        ('lib/' + package_name, [package_name+'/FindPath.py']),
        ('lib/' + package_name, [package_name+'/OR.py']),
        ('lib/' + package_name, [package_name+'/ORExport.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csl',
    maintainer_email='csl@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = py_srvcli.service_member_function:main',
            'client = py_srvcli.client_member_function:main'
        ],
    },
)
