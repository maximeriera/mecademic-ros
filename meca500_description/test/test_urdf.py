import os
import xml.dom.minidom
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory


def test_urdf_parsing():
    # Find the xacro file
    package_name = 'meca500_description'
    try:
        share_path = get_package_share_directory(package_name)
    except Exception:
        # Fallback for local testing if not installed
        share_path = os.path.join(os.getcwd(), package_name)

    xacro_file = os.path.join(share_path, 'urdf', 'meca500.urdf.xacro')

    if not os.path.exists(xacro_file):
        pytest.fail(f'Xacro file not found at {xacro_file}')

    # Process xacro
    mappings = {'robot_ip': '192.168.0.100'}
    doc = xacro.process_file(xacro_file, mappings=mappings)
    urdf_xml = doc.toprettyxml(indent='  ')

    # Check if it's a valid XML
    try:
        xml.dom.minidom.parseString(urdf_xml)
    except Exception as e:
        pytest.fail(f'Failed to parse generated URDF as XML: {e}')

    # Check for some expected elements
    assert 'robot' in urdf_xml
    assert 'meca_axis_1' in urdf_xml
    assert 'meca500_hardware/Meca500SystemHardware' in urdf_xml
    assert '192.168.0.100' in urdf_xml


if __name__ == '__main__':
    test_urdf_parsing()
