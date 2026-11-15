"""Neutral link frames from a prebuilt URDF, without a physics or model builder import."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def neutral_frame(urdf,link):
    root=ET.parse(urdf).getroot()
    parents={joint.find('child').get('link'):joint for joint in root.findall('joint')}
    chain=[];seen=set()
    while link in parents:
        if link in seen:raise ValueError('cyclic URDF link graph')
        seen.add(link);joint=parents[link];chain.append(joint);link=joint.find('parent').get('link')
    result=np.eye(4)
    for joint in reversed(chain):
        element=joint.find('origin');step=np.eye(4)
        if element is not None:
            step[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
            step[:3,:3]=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
        result=result@step
    return result
