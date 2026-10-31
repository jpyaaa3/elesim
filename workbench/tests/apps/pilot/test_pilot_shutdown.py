import subprocess
import sys
import textwrap


def test_sigterm_unwinds_runtime_and_restores_signal_handler():
    script = textwrap.dedent('''
        import os,signal
        from elesim_pilot.shutdown import graceful_sigterm
        previous=signal.getsignal(signal.SIGTERM)
        try:
            with graceful_sigterm():
                print('started',flush=True)
                try:os.kill(os.getpid(),signal.SIGTERM)
                finally:print('runtime-cleanup',flush=True)
        except SystemExit as error:
            assert error.code==0
            assert signal.getsignal(signal.SIGTERM)==previous
            print('handler-restored',flush=True)
    ''')
    result = subprocess.run([sys.executable,'-c',script],capture_output=True,text=True,timeout=15)
    assert 'started' in result.stdout,result.stderr
    assert result.returncode==0,(result.returncode,result.stdout,result.stderr)
    assert all(marker in result.stdout for marker in ('runtime-cleanup','handler-restored'))
