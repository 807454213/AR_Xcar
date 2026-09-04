# Third-Party Licenses and Assets

Xcar2 uses third-party SDKs, libraries, model runtimes and model artifacts. Before publishing a release, verify the exact source and license of each bundled asset.

## Source and SDK Components

Known third-party areas include:

- `AI/base/rknpu2/`: Rockchip RKNN runtime, examples and documentation.
- `AI/PPOCR-1/`: PaddleOCR-related code and model runtime glue.
- `AI/PPOCR-1/PPOCR-Det/3rdparty/`: bundled third-party dependencies such as OpenCV, libjpeg-turbo, librga and OpenCL stubs.
- System libraries linked by CMake: OpenCV, RKNN runtime, pthread, rt, RGA, turbojpeg, curl, OpenSSL and crypto.

Keep each dependency's original license file with the dependency, and add release-specific notices when distributing binary builds.

## Model Artifacts

The repository may reference RKNN model files for YOLO, PPSeg and OCR. These files can be large and may have separate training-data or redistribution restrictions.

Recommended release policy:

- Do not commit private or uncleared model weights to the public repository.
- Publish redistributable models through GitHub Releases or Git LFS only after confirming their license.
- Document each model's purpose, input size, source, conversion process and license.
- If a model cannot be redistributed, keep only the expected path and conversion/download instructions.

## Competition Documents

Competition manuals and rule documents may be copyrighted by their organizer. Prefer linking to the official source unless redistribution permission is clear.
