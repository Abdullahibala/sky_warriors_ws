from setuptools import setup

package_name = 'cbba_task_allocator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboticistprogrammer',
    maintainer_email='user@example.com',
    description='Leader-assisted CBBA task allocation for swarm UAVs.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cbba_leader_node = cbba_task_allocator.cbba_leader_node:main',
            'cbba_agent_node = cbba_task_allocator.cbba_agent_node:main',
            'task_adapter_qr = cbba_task_allocator.task_adapter_qr:main',
            'agent_state_node = cbba_task_allocator.agent_state_node:main',
            'task_executor_node = cbba_task_allocator.task_executor_node:main',
        ],
    },
)
