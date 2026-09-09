"""Import STEP assembly placement into a static, visual-only MuJoCo scene.

Requires cadquery-ocp. STEP units are converted by OCCT to mm, then meshes to m.
No joint axes, masses or collision shapes are inferred from assembly names.
"""
import argparse
import json
import re
import struct
from pathlib import Path
from xml.etree import ElementTree as ET

from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder


def split_stl(path):
    """Respect MuJoCo's 200,000-triangle limit per binary STL asset."""
    raw = path.read_bytes()
    count = struct.unpack_from("<I", raw, 80)[0]
    if count <= 190000:
        return [path]
    parts = []
    for index, start in enumerate(range(0, count, 190000)):
        end = min(start + 190000, count)
        part = path.with_stem(f"{path.stem}_{index}")
        part.write_bytes(raw[:80] + struct.pack("<I", end-start) + raw[84+start*50:84+end*50])
        parts.append(part)
    path.unlink()
    return parts


def name(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else "unnamed"


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "cad_300mm")
    parser.add_argument("--inspect-axes", action="store_true", help="export cylinder geometry only")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    print("Reading STEP", flush=True)
    if reader.ReadFile(str(args.source)) != IFSelect_RetDone:
        raise RuntimeError("STEP read failed")
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    if not reader.Transfer(doc):
        raise RuntimeError("STEP transfer failed")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    builder = BRep_Builder()
    groups, inventory = {}, []
    bounds = Bnd_Box()

    def walk(label, location, path):
        current_name = name(label)
        location = location.Multiplied(tool.GetLocation_s(label))
        referred = TDF_Label()
        if tool.GetReferredShape_s(label, referred):
            label = referred
        path = path + [current_name]
        children = TDF_LabelSequence()
        if tool.GetComponents_s(label, children):
            for i in range(1, children.Length() + 1):
                walk(children.Value(i), location, path)
            return
        shape = tool.GetShape_s(label)
        if shape.IsNull():
            return
        shape = shape.Located(location)
        group = "body"
        for index, part in enumerate(path):
            leg = re.search(r"(?:^|[-_])(FL|FR|RL|RR)(?:$|[_ ])", part)
            if leg:
                joint = next((re.match(r"J([123])(?:\s|_|$)", p) for p in path[index+1:] if re.match(r"J([123])(?:\s|_|$)", p)), None)
                group = leg.group(1).lower() + ("_j" + joint.group(1) if joint else "_fixed")
                break
        if group not in groups:
            compound = TopoDS_Compound()
            builder.MakeCompound(compound)
            groups[group] = compound
        builder.Add(groups[group], shape)
        BRepBndLib.AddOptimal_s(shape, bounds)
        inventory.append({"path": path, "mesh_group": group})

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    print(f"Imported {len(inventory)} leaf components, {len(groups)} mesh groups", flush=True)
    if args.inspect_axes:
        cylinders = {}
        for group, shape in groups.items():
            if group == 'body':
                continue
            entries = []
            faces = TopExp_Explorer(shape, TopAbs_FACE)
            while faces.More():
                surface = BRepAdaptor_Surface(TopoDS.Face_s(faces.Current()))
                if surface.GetType() == GeomAbs_Cylinder:
                    cylinder = surface.Cylinder()
                    entries.append(dict(radius_mm=cylinder.Radius(), origin_mm=list(cylinder.Location().Coord()), axis=list(cylinder.Axis().Direction().Coord())))
                faces.Next()
            cylinders[group] = entries
        (args.output / 'cylinder_candidates.json').write_text(json.dumps(cylinders, indent=2))
        print('Cylinder geometry saved; these are candidates, not certified joints.', flush=True)
        return
    extent = list(bounds.Get())
    center = [(extent[i] + extent[i+3]) / 2000 for i in range(3)]
    model = ET.Element("mujoco", model="Spot OMG new STEP — static CAD preview")
    ET.SubElement(model, "compiler", angle="radian")
    ET.SubElement(model, "option", gravity="0 0 0")
    ET.SubElement(model, "statistic", center=" ".join(map(str, center)), extent=str(max(extent[i+3]-extent[i] for i in range(3))/1000))
    asset = ET.SubElement(model, "asset")
    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "light", pos="0 0 2", directional="true", dir="0 0 -1")
    colors = {"fl": "0.25 0.6 0.9 1", "fr": "0.25 0.8 0.6 1", "rl": "0.95 0.65 0.25 1", "rr": "0.8 0.4 0.7 1"}
    for group, shape in sorted(groups.items()):
        print(f"Meshing {group}", flush=True)
        BRepMesh_IncrementalMesh(shape, 0.3, False, 0.35, True).Perform()
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        if not writer.Write(shape, str(args.output / f"{group}.stl")):
            raise RuntimeError(f"Mesh export failed: {group}")
        for part in split_stl(args.output / f"{group}.stl"):
            ET.SubElement(asset, "mesh", name=part.stem, file=part.name, scale="0.001 0.001 0.001")
            ET.SubElement(world, "geom", name=part.stem, type="mesh", mesh=part.stem, contype="0", conaffinity="0", rgba=colors.get(group[:2], "0.65 0.68 0.72 1"))
    ET.indent(model)
    ET.ElementTree(model).write(args.output / "scene.xml", encoding="unicode")
    report = {"source": str(args.source.resolve()), "mode": "static visual-only; no articulated physics", "central_aluminum_frame_length_m": 0.3, "cad_bounds_mm": extent, "cad_envelope_xyz_mm": [extent[i+3]-extent[i] for i in range(3)], "leaf_count": len(inventory), "groups": sorted(groups), "components": inventory}
    (args.output / "assembly.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k != "components"}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
