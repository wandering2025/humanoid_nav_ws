"""Read-only Windows event and appended WSLg-error checks; no settings changes."""
import json
from pathlib import Path
import subprocess

POWERSHELL='/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe'


def events(after=None):
    script="$ErrorActionPreference='Stop'; $rows=@(); $last=@{}; "
    for name in ('System','Application'):
        script+=f"$last['{name}']=(Get-WinEvent -LogName '{name}' -MaxEvents 1).RecordId; "
        if after is not None:
            record=int(after[name])
            script+=f"try {{ $es=Get-WinEvent -LogName '{name}' -FilterXPath '*[System[EventRecordID > {record}]]' -ErrorAction Stop; "
            script+="foreach($e in $es) { if($e.ProviderName -eq 'Display' -or $e.ProviderName -eq 'nvlddmkm' -or ($e.ProviderName -eq 'Microsoft-Windows-Kernel-Power' -and $e.Id -eq 41) -or ($e.Id -eq 1001 -and $e.Message -match 'LiveKernelEvent')) { $rows += [pscustomobject]@{RecordId=$e.RecordId;Id=$e.Id;Provider=$e.ProviderName;Message=$e.Message} } } } catch { if($_.FullyQualifiedErrorId -notmatch 'NoMatchingEventsFound') {throw} }; "
    script+="[pscustomobject]@{Last=$last;Events=@($rows)} | ConvertTo-Json -Depth 5 -Compress"
    p=subprocess.run([POWERSHELL,'-NoProfile','-NonInteractive','-Command',script],capture_output=True,timeout=5)
    if p.returncode:raise RuntimeError(f'Windows event query failed: {p.stderr.decode(errors="replace")}')
    return json.loads(p.stdout.decode('utf-8',errors='replace').lstrip('\ufeff'))


class HostGraphics:
    def __init__(self,out):
        self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
        self.baseline=events()['Last'];self.checks=[]
        self.logs={}
        for name in ('weston.log','stderr.log'):
            path=Path('/mnt/wslg')/name
            data=path.read_bytes();self.logs[path]=len(data)
            (self.out/(name+'.baseline')).write_bytes(data)
        (self.out/'windows-baseline.json').write_text(json.dumps(self.baseline,indent=2)+'\n')

    def check(self,pattern,*,windows=False):
        for path,offset in list(self.logs.items()):
            data=path.read_bytes()
            if len(data)<offset:raise RuntimeError(f'WSLg log rotated/truncated during test: {path}')
            delta=data[offset:];self.logs[path]=len(data)
            if delta:
                with (self.out/(path.name+'.delta')).open('ab') as f:f.write(delta)
                if pattern.search(delta.decode(errors='replace')):raise RuntimeError(f'New WSLg graphics error: {path}')
        if windows:
            value=events(self.baseline);self.checks.append(value)
            (self.out/'windows-checks.json').write_text(json.dumps(self.checks,indent=2)+'\n')
            if value['Events']:raise RuntimeError('New Windows display/driver/power event; see windows-checks.json')
