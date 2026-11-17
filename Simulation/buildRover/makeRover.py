"""Build the simulation USD from lunabot_2627.urdf and its meshes."""

from pathlib import Path
import xml.etree.ElementTree as ET


BUILD_DIR = Path(__file__).resolve().parent
URDF_PATH = BUILD_DIR / "lunabot_2627.urdf"
USD_DIR = BUILD_DIR.parent / "assets" / "lunabot_2627"
ROVER_USD = USD_DIR / "lunabot_2627.usda"


def build_rover():
    # Check all inputs before the converter can create an incomplete asset.
    robot = ET.parse(URDF_PATH).getroot()
    missing = sorted({
        mesh.attrib["filename"]
        for mesh in robot.findall(".//mesh")
        if not (BUILD_DIR / mesh.attrib["filename"]).is_file()
    })
    if missing:
        raise FileNotFoundError("Missing URDF meshes relative to " + str(BUILD_DIR) + ":\n" + "\n".join(missing))

    from urdf_usd_converter import Converter
    from pxr import Usd

    # Isaac Lab supplies the world/physics scene; PreviewSurface works with the CPU renderer.
    asset = Converter(scene=False, openpbr=False).convert(str(URDF_PATH), str(USD_DIR))
    output = Path(asset.path)
    if output.resolve() != ROVER_USD.resolve() or not output.is_file():
        raise RuntimeError(f"Expected converted rover at {ROVER_USD}, got {output}")

    # The converter anchors the root to the world by default. This rover must drive freely.
    stage = Usd.Stage.Open(str(output))
    root_joint = stage.GetPrimAtPath(stage.GetDefaultPrim().GetPath().AppendPath("Physics/root_joint"))
    if not root_joint:
        raise RuntimeError("Converted USD is missing the expected root joint")
    root_joint.SetActive(False)
    stage.GetRootLayer().Save()
    return output


def main():
    print(f"Created: {build_rover()}")


if __name__ == "__main__":
    main()
