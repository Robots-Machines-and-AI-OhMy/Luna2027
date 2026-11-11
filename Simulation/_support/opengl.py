"""Select software OpenGL before Newton or pyglet can load a graphics driver."""

import ctypes
import os
from pathlib import Path
import sys

_dll_directory = None
_opengl_library = None


def prepare_software_opengl():
    global _dll_directory, _opengl_library
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
    os.environ["GALLIUM_DRIVER"] = "llvmpipe"
    if sys.platform == "win32":
        mesa = Path(sys.prefix) / "mesa" / "x64"
        library = mesa / "opengl32.dll"
        if not library.is_file():
            raise RuntimeError(
                "Software OpenGL is missing. Run uv run setup.py from the Simulation folder first."
            )
        # Keep these handles alive. Preloading this exact DLL makes subsequent
        # opengl32 loads use Mesa instead of the system's GPU-backed OpenGL.
        _dll_directory = os.add_dll_directory(str(mesa))
        _opengl_library = ctypes.WinDLL(str(library))
        # PyOpenGL resolves an absolute filename through ctypes.util.find_library,
        # which searches PATH rather than the DLL directories registered above.
        os.environ["PATH"] = str(mesa) + os.pathsep + os.environ.get("PATH", "")

    import pyglet

    # Newton creates the actual window. Avoid an extra hidden OpenGL window
    # whose callbacks can outlive the native viewer during Python shutdown.
    pyglet.options["shadow_window"] = False


def verify_software_opengl():
    """Verify the renderer of the actual initialized Newton window."""
    from pyglet import gl

    renderer = gl.gl_info.get_renderer()
    if "llvmpipe" not in renderer.lower():
        raise RuntimeError(f"Expected CPU software OpenGL (llvmpipe), got {renderer!r}; refusing GPU rendering.")
    print(f"CPU_OPENGL_RENDERER={renderer}", flush=True)
    return renderer


def close_windows():
    """Close native windows while their Python event handlers are still alive."""
    from pyglet import app

    for window in list(app.windows):
        window.close()
