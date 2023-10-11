import io
from typing import Union
import xml.etree.ElementTree as ET


class SDFUtil:

    @staticmethod
    def parse(sdf: str) -> ET.ElementTree:
        file = io.StringIO(sdf)
        xml = ET.parse(file)
        return xml

    @staticmethod
    def serialize(sdf: ET.ElementTree) -> str:
        file = io.StringIO()
        sdf.write(file, encoding="Unicode", xml_declaration=True)
        return file.getvalue()

    @staticmethod
    def get_model_root(sdf: ET.ElementTree, tag="model") -> Union[ET.Element, None]:
        root = sdf.getroot()
        if root.tag != tag:
            root = root.find(tag)

        return root

    @staticmethod
    def set_name(sdf: ET.ElementTree, name: str, tag="model") -> None:
        root = SDFUtil.get_model_root(sdf, tag)

        #TODO reconsider whether this should fail silently
        if root is not None:
            root.set("name", name)


    SFM_PLUGIN_SELECTOR = r"""plugin[@filename='libPedestrianSFMPlugin.so']"""

    @staticmethod
    def delete_all(sdf: ET.ElementTree, selector:str) -> int:
        hits = 0
        for plugin_parent in sdf.findall(f".//{selector}/.."):
            for plugin in plugin_parent.findall(f"./{selector}"):
                plugin_parent.remove(plugin)
                hits += 1
        
        return hits