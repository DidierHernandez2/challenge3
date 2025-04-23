from setuptools import setup

package_name = 'control_trayectoria'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/control.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TuNombre',
    maintainer_email='tunombre@correo.com',
    description='Controlador PID para trayectoria cuadrada en Puzzlebot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_control = control_trayectoria.pid_control:main',
        ],
    },
)
