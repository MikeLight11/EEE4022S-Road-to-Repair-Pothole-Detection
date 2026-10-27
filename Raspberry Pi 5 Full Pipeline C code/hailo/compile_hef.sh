#!/usr/bin/env bash
# Compile best_seg.pt (YOLO11n-seg) into a Hailo-8 HEF with Ultralytics' Hailo export.
#
# Needs Linux x86_64 (the Dataflow Compiler doesn't run on macOS or ARM) and,
# for full accuracy, an NVIDIA GPU: without one the DFC lowers the optimisation
# level. Google Colab with a T4 runtime works - see compile_hef_colab.ipynb.
#
#   ./compile_hef.sh best_seg.pt calib.zip hailo_dataflow_compiler-3.33.0-py3-none-linux_x86_64.whl
#
# The DFC version must match the Pi's HailoRT: HailoRT 4.23 (hailo-all on
# Raspberry Pi OS Trixie) pairs with DFC 3.33. Download the wheel from the Hailo
# Developer Zone (free account): Software Downloads -> Dataflow Compiler.
#
# Quantisation-aware fine-tuning is off by default: on this model its distillation
# loss rose every epoch (worst on the class-score outputs), i.e. it pulled the INT8
# model away from the float one. FINETUNE=1 turns it back on for comparison.
#
# A16_CLS=1 runs the three class-score output convs (conv60/74/92) at 16-bit
# activations instead of 8: INT8 squashed borderline pothole confidences (0.4-0.6 in
# PyTorch came out ~0.2 on the Hailo), which costs tracks their 5 confident hits.
#
# HAILO_ARCH=hailo8l compiles for the 13 TOPS Hailo-8L instead (the HEF also runs
# on a Hailo-8, which is how the 8L is approximated for the report's comparison).
#
# OPTIMIZE_ONLY=1 stops after quantisation, saving the quantised model as a HAR
# (minutes instead of an hour+): eval_quant.py emulates it to compare variants, and
# compile_from_har.py compiles the chosen one without quantising again.
#
# Output: hef_build/<variant>.har always (the quantised model), and unless
#         OPTIMIZE_ONLY=1, <variant>.zip (best_seg.hef + metadata.yaml); <variant> is
#         best_seg_hailo_model with _hailo8l, _finetune and/or _a16cls appended
set -euo pipefail

MODEL="$(realpath "${1:?usage: $0 best_seg.pt calib.zip dfc.whl}")"
CALIB="$(realpath "${2:?calibration zip from make_calibration_set.py}")"
WHEEL="$(realpath "${3:?Hailo Dataflow Compiler wheel}")"
WORK="${WORK:-$PWD/hef_build}"
FINETUNE="${FINETUNE:-0}"
HAILO_ARCH="${HAILO_ARCH:-hailo8}"
A16_CLS="${A16_CLS:-0}"
OPTIMIZE_ONLY="${OPTIMIZE_ONLY:-0}"   # 1: quantise and save the HAR, skip the (slow) compile
VARIANT=best_seg_hailo_model
[ "$HAILO_ARCH" = hailo8 ] || VARIANT="${VARIANT}_$HAILO_ARCH"
[ "$FINETUNE" = 1 ] && VARIANT="${VARIANT}_finetune"
[ "$A16_CLS" = 1 ] && VARIANT="${VARIANT}_a16cls"
case "$HAILO_ARCH" in hailo8 | hailo8l) ;; *) echo "HAILO_ARCH must be hailo8 or hailo8l"; exit 1 ;; esac
ULTRALYTICS_VERSION=8.4.153   # the export code this was checked against
export USER="${USER:-$(id -un)}"   # the DFC's compile step reads $USER, which Colab leaves unset

[ "$(uname -s)-$(uname -m)" = Linux-x86_64 ] || { echo "needs Linux x86_64"; exit 1; }
mkdir -p "$WORK"

echo "== system packages (graphviz is a DFC dependency)"
if command -v apt-get >/dev/null; then
    SUDO=""; [ "$(id -u)" -eq 0 ] || SUDO=sudo
    $SUDO apt-get update -qq
    $SUDO apt-get install -y -qq graphviz libgraphviz-dev >/dev/null
fi

case "$(basename "$WHEEL")" in
    hailo_dataflow_compiler-3.33.*) ;;
    *) echo "WARNING: $(basename "$WHEEL") is not DFC 3.33. The Pi runs HailoRT 4.23, which pairs with"
       echo "         DFC 3.33; a HEF from a newer compiler may fail to load ('Unsupported hef version')." ;;
esac

echo "== Python 3.10 environment (works with every DFC 3.x release)"
command -v uv >/dev/null || { curl -LsSf https://astral.sh/uv/install.sh | sh; }
export PATH="$HOME/.local/bin:$PATH"
# Colab sets UV_SYSTEM_PYTHON=1, which makes `uv pip` install into the system Python
# (3.13, too new for the DFC's TensorFlow) instead of this venv.
unset UV_SYSTEM_PYTHON
PY="$WORK/venv/bin/python"
# Reuse the environment from an earlier variant in the same session (saves a few minutes).
if [ -x "$PY" ] && "$PY" -c "import hailo_sdk_client, ultralytics" 2>/dev/null; then
    echo "   reusing $WORK/venv"
    # shellcheck disable=SC1091
    . "$WORK/venv/bin/activate"
else
    uv venv --seed --python 3.10 "$WORK/venv"
    # shellcheck disable=SC1091
    . "$WORK/venv/bin/activate"

    echo "== Dataflow Compiler"
    uv pip install --python "$PY" "$WHEEL"
    TF_VERSION="$("$PY" -c 'import importlib.metadata as m; print(m.version("tensorflow"))')"
    # The DFC pins a CPU TensorFlow; the [and-cuda] extra of the same version adds the CUDA libraries.
    uv pip install --python "$PY" "tensorflow[and-cuda]==$TF_VERSION" || echo "warning: could not add CUDA TensorFlow"

    echo "== Ultralytics (CPU torch: only used to export the ONNX)"
    uv pip install --python "$PY" torch torchvision --index-url https://download.pytorch.org/whl/cpu
    uv pip install --python "$PY" "ultralytics==$ULTRALYTICS_VERSION" onnx onnxslim onnxruntime
fi

GPUS="$("$PY" -c 'import tensorflow as tf; print(len(tf.config.list_physical_devices("GPU")))' 2>/dev/null || echo 0)"
if [ "$GPUS" -gt 0 ]; then
    echo "== TensorFlow sees $GPUS GPU(s)"
else
    echo "== WARNING: TensorFlow sees no GPU. The DFC will lower the optimisation level,"
    echo "   so the HEF will be less accurate. On Colab: Runtime -> Change runtime type -> T4 GPU."
    if [ "$FINETUNE" = 1 ]; then
        echo "== ERROR: FINETUNE=1 needs a GPU (without one the DFC skips fine-tuning silently). Stopping."
        exit 1
    fi
fi

echo "== calibration set"
rm -rf "$WORK/calib"
mkdir -p "$WORK/calib"
unzip -q "$CALIB" -d "$WORK/calib"
echo "$(ls "$WORK/calib/images/val" | wc -l) images"

echo "== export for $HAILO_ARCH"
cp "$MODEL" "$WORK/best_seg.pt"
cd "$WORK"
echo "RAM: $(free -g | awk '/^Mem:/ {print $2 " GB total, " $7 " GB available"}') (DFC asks for 16 GB)"
"$PY" - <<EOF
from pathlib import Path

from hailo_sdk_client import ClientRunner
from ultralytics import YOLO

# Ultralytics writes the model script itself; append to it here (a later command
# overrides an earlier one). The layer noise analysis is only a per-layer accuracy
# report (it doesn't change the HEF) and it exhausts a standard 12 GB Colab runtime.
_load_model_script = ClientRunner.load_model_script
def load_model_script(self, script, *args, **kwargs):
    script += "\nmodel_optimization_config(checker_cfg, policy=disabled)"
    # The C2PSA attention matmul feeds both halves of the softmax (max and subtract),
    # and the Hailo-8 compiler can't place a matmul with two outputs ("More than one
    # output is not supported for layer matmul1"). A shortcut layer gives it a single
    # consumer. Both lines are from Hailo's own Model Zoo yolov11n_seg.alls (v2.19+),
    # which uses the same end nodes and layer names as this export.
    script += "\nshortcut1 = shortcut(matmul1, [reduce_max_softmax1, ew_sub_softmax1])"
    script += "\nallocator_param(automatic_reshapes=disabled)"
    if "$FINETUNE" != "1":
        script += "\npost_quantization_optimization(finetune, policy=disabled)"
    if "$A16_CLS" == "1":
        # Class scores at 16-bit: the outputs whose INT8 rounding hurt detection most.
        script += "\nquantization_param([conv60, conv74, conv92], precision_mode=a16_w16)"
    print("model script:\n" + script)
    return _load_model_script(self, script, *args, **kwargs)
ClientRunner.load_model_script = load_model_script

# Save the quantised model before the slow compile, so it can be emulated
# (eval_quant.py) or compiled later (compile_from_har.py) without quantising again.
_compile = ClientRunner.compile
def compile(self, *args, **kwargs):
    har = Path("$WORK/$VARIANT.har")
    self.save_har(str(har))
    print(f"saved the quantised model to {har}", flush=True)
    if "$OPTIMIZE_ONLY" == "1":
        raise SystemExit(0)
    return _compile(self, *args, **kwargs)
ClientRunner.compile = compile

model = YOLO("best_seg.pt")
assert model.task == "segment", model.task
calib = Path("calib").resolve()
# Images only, no labels: the exporter only needs them for quantisation ranges.
(calib / "data.yaml").write_text(
    f"path: {calib}\ntrain: images/val\nval: images/val\nnames: {dict(model.names)}\n")
out = model.export(format="hailo", name="$HAILO_ARCH", imgsz=640, data=str(calib / "data.yaml"))
print("exported:", out)
EOF

if [ "$OPTIMIZE_ONLY" = 1 ]; then
    [ -f "$WORK/$VARIANT.har" ] || { echo "no HAR was saved"; exit 1; }
    echo
    echo "Done (quantised only): $WORK/$VARIANT.har"
    exit 0
fi
ZIP="$VARIANT.zip"
rm -f "$ZIP"
(cd best_seg_hailo_model && zip -q "../$ZIP" ./*)
echo
echo "Done: $WORK/$ZIP"
unzip -l "$ZIP"
