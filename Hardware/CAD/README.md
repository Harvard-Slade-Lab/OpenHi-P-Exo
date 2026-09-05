# CAD files

This folder contains the parts designed for the OpenHi-P exoskeleton: the custom
SolidWorks parts, the `DWG/` drawings for machined parts, and the `STL/` files for
printed parts and molds.

## Commercial off-the-shelf parts are not included

The fasteners and the actuator model are commercial parts supplied by their vendors,
so their CAD files are not redistributed here. `Assmebly.SLDASM` still references them,
and SolidWorks will report them as missing until you supply them yourself.

To open the full assembly, download each part from its vendor and save it into this
folder. **SolidWorks resolves references by file name, so save each file under exactly
the name listed below** and the assembly will rebuild without further edits.

### McMaster-Carr fasteners

Download from [mcmaster.com](https://www.mcmaster.com) by searching the part number.

| Part number | Description | Save as |
|---|---|---|
| 91116A350 | 18-8 Stainless Steel Oversized Washer | `91116A350_18-8 Stainless Steel Oversized Washer.SLDPRT` |
| 91116A360 | 18-8 Stainless Steel Oversized Washer | `91116A360_18-8 Stainless Steel Oversized Washer.SLDPRT` |
| 91239A144 | Button Head Hex Drive Screw | `91239A144_Button Head Hex Drive Screw.SLDPRT` |
| 91239A232 | Button Head Hex Drive Screw | `91239A232_Button Head Hex Drive Screw.SLDPRT` |
| 91239A318 | Button Head Hex Drive Screw | `91239A318_Button Head Hex Drive Screw.SLDPRT` |
| 91251A345 | Black-Oxide Alloy Steel Socket Head Screw | `91251A345_Black-Oxide Alloy Steel Socket Head Screw.SLDPRT` |
| 91253A197 | Black-Oxide Alloy Steel Hex Drive Flat Head Screw | `91253A197_Black-Oxide Alloy Steel Hex Drive Flat Head Screw.SLDPRT` |
| 91294A192 | Black-Oxide Alloy Steel Hex Drive Flat Head Screw | `91294A192_Black-Oxide Alloy Steel Hex Drive Flat Head Screw.SLDPRT` |
| 94645A102 | High-Strength Steel Nylon-Insert Locknut | `94645A102_High-Strength Steel Nylon-Insert Locknut.SLDPRT` |
| 95947A560 | Aluminum Female Threaded Hex Standoff | `95947A560_Aluminum Female Threaded Hex Standoff.SLDPRT` |
| 97131A120 | Medium-Strength Nylon-Insert Locknut | `97131A120_Medium-Strength Nylon-Insert Locknut.SLDPRT` |
| 98093A211 | Black-Phosphate Class 10.9 Steel Hex Head Screw | `98093A211_Black-Phosphate Class 10.9 Steel Hex Head Screw.SLDPRT` |
| 98093A212 | Black-Phosphate Class 10.9 Steel Hex Head Screw | `98093A212_Black-Phosphate Class 10.9 Steel Hex Head Screw.SLDPRT` |
| 98093A310 | Black-Phosphate Class 10.9 Steel Screw | `98093A310_Black-Phosphate Class 10.9 Steel Screw.SLDPRT` |
| 98093A316 | Black-Phosphate Class 10.9 Steel Hex Head Screw | `98093A316_Black-Phosphate Class 10.9 Steel Hex Head Screw.SLDPRT` |
| 98370A009 | 18-8 Stainless Steel Oversized Washer | `98370A009_18-8 Stainless Steel Oversized Washer.SLDPRT` |

### Actuator

| Part | Source | Save as |
|---|---|---|
| CubeMars AK10-9 V2.0 | CubeMars product page | `AK10-9.SLDPRT` |

Quantities and the rest of the bill of materials are in
`../Hip Exoskeleton Hardware Building Guide.pdf`.
