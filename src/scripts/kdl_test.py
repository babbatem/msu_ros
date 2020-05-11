from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

# robot = URDF.load_from_parameter_server(verbose=False)
path = '/home/abba/kinova-ros/kinova_description/urdf/jaco6.urdf'
robot = URDF.from_xml_file(path)

base_link = "world"
end_link = "j2s6s300_link_6"

tree = kdl_tree_from_urdf_model(robot)
chain = tree.getChain(base_link, end_link)
for i in range(chain.getNrOfSegments()):
    print(chain.getSegment(i).getName())# print(chain)
