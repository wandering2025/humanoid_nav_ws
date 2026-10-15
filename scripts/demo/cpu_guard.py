"""One-way, per-process GPU device denial before importing ROS/MuJoCo/Torch.

Linux Landlock ABI >= 5. No global permissions, mounts or drivers are changed.
All exec/fork children inherit restrictions. No GPU device is opened to test it.
"""
import ctypes
import errno
import hashlib
import json
import os
from pathlib import Path
import stat

READ_FILE = 1 << 2
WRITE_FILE = 1 << 1
IOCTL_DEV = 1 << 15
HANDLED = READ_FILE | WRITE_FILE | IOCTL_DEV
SAFE_DEVICES = ('null', 'zero', 'random', 'urandom', 'tty', 'pts', 'shm', 'mqueue')


class Ruleset(ctypes.Structure):
    _fields_ = [('handled_access_fs', ctypes.c_uint64)]


class PathRule(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('allowed_access', ctypes.c_uint64), ('parent_fd', ctypes.c_int32)]


def enter(*, software_display=False):
    if os.uname().machine != 'x86_64':
        raise RuntimeError('CPU guard supports only verified x86_64 syscall numbers; refusing.')
    libc = ctypes.CDLL(None, use_errno=True)

    def syscall(number, *args):
        value = libc.syscall(number, *args)
        if value < 0:
            code = ctypes.get_errno()
            raise OSError(code, os.strerror(code))
        return value

    abi = syscall(444, None, 0, 1)
    if abi < 5:
        raise RuntimeError(f'Landlock ABI {abi} lacks device ioctl restrictions; refusing.')
    # Existing device FDs escape Landlock's open-time checks. Refuse unknown ones.
    for entry in Path('/proc/self/fd').iterdir():
        try:
            info = os.fstat(int(entry.name))
            target = os.readlink(entry)
        except OSError:
            continue
        if stat.S_ISCHR(info.st_mode) or stat.S_ISBLK(info.st_mode):
            if target not in ('/dev/null', '/dev/zero', '/dev/random', '/dev/urandom', '/dev/tty') and not target.startswith('/dev/pts/'):
                raise RuntimeError(f'Unexpected inherited device FD {entry.name}: {target}')
    # /dev is not granted as a parent: no GPU node, current or future, is allowed.
    paths = [f'/{name}' for name in ('bin', 'sbin', 'usr', 'lib', 'lib64', 'etc',
             'home', 'opt', 'tmp', 'var', 'proc', 'sys', 'run') if Path('/',name).exists()]
    paths += [f'/dev/{name}' for name in SAFE_DEVICES if Path('/dev', name).exists()]
    ruleset = Ruleset(HANDLED)
    fd = syscall(444, ctypes.byref(ruleset), ctypes.sizeof(ruleset), 0)
    try:
        for path in paths:
            parent = os.open(path, os.O_PATH | os.O_CLOEXEC)
            try:
                rule = PathRule(HANDLED, parent)
                syscall(445, fd, 1, ctypes.byref(rule), 0)
            finally:
                os.close(parent)
        if libc.prctl(38, 1, 0, 0, 0) != 0:  # PR_SET_NO_NEW_PRIVS
            raise OSError(ctypes.get_errno(), 'no_new_privs failed')
        syscall(446, fd, 0)
    finally:
        os.close(fd)
    # Prove enforcement against a harmless, deliberately ungranted device.
    # Do NOT test /dev/dxg or GPU ioctls: an incorrect guard must not touch them.
    try:
        probe = os.open('/dev/full', os.O_RDONLY)
    except OSError as exc:
        if exc.errno != errno.EACCES:
            raise RuntimeError(f'Unexpected denial result: {exc}') from exc
    else:
        os.close(probe)
        raise RuntimeError('Device denial not enforced; refusing all runtime imports.')
    with open('/dev/null', 'rb') as allowed:
        allowed.read(0)
    display=os.environ.get('DISPLAY','') if software_display else ''
    wayland=os.environ.get('WAYLAND_DISPLAY','') if software_display else ''
    os.environ.update(CUDA_VISIBLE_DEVICES='', NVIDIA_VISIBLE_DEVICES='none',
                      LIBGL_ALWAYS_SOFTWARE='true', GALLIUM_DRIVER='llvmpipe',
                      LP_NUM_THREADS='2', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
                      DISPLAY=display, WAYLAND_DISPLAY=wayland, WSL_INTEROP='', QT_OPENGL='software')
    # Scheduler niceness affects only this process and its descendants.
    if os.getpriority(os.PRIO_PROCESS, 0) < 10:
        os.nice(10 - os.getpriority(os.PRIO_PROCESS, 0))
    return dict(landlock_abi=abi, handled_access_fs=HANDLED, allowed_paths=sorted(paths),
                forbidden_probe='/dev/full', forbidden_probe_errno=errno.EACCES,
                no_gpu_device_opened_for_testing=True, no_new_privs=libc.prctl(39, 0, 0, 0, 0),
                renderer='software-only' if software_display else 'disabled', cuda_probe=False, nice=os.getpriority(os.PRIO_PROCESS, 0),
                implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


if __name__ == '__main__':
    report = enter()
    print(json.dumps(report, indent=2))
