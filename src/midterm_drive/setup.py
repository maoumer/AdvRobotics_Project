from setuptools import setup

package_name = 'midterm_drive'

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
    maintainer='Maleficent',
    maintainer_email='root@todo.todo',
    description='Midterm Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                 'talker = midterm_drive.midterm_proj:main'
        ],
    },
)
