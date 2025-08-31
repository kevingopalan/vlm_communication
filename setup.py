from setuptools import find_packages, setup

package_name = 'vlm_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Gopalan',
    maintainer_email='kevin.gopalan@gmail.com',
    description='Helper for transferring robot data to the server and VLM for data processing using rclpy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'robot_publisher_script = vlm_communication.publisher_member_function:main',
                'server_subscriber_script = vlm_communication.subscriber_member_function:main',
                'genmapimg = vlm_communication.genmapimg:main',
                'genmapimg_with_vlm = vlm_communication.genmapimg_with_vlm:main',
        ],
    },
)
