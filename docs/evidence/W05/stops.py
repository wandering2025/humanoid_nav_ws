import csv,collections,sys
rows=list(csv.DictReader(open(sys.argv[1])))
cmd={(r['trial'],r['rep']):r for r in rows if r['segment']=='1'}
f=lambda r,k: float(r[k])
g=collections.defaultdict(list)
for r in rows:
    if r['segment']=='2' and r.get('stop_time_s'):
        c=cmd[(r['trial'],r['rep'])]
        fam=r['family']
        if fam.startswith('grid'): key=f"walk ux={float(c['ux']):.2f}"
        elif fam=='inplace': key=f"inplace |uwz|={abs(float(c['uwz'])):.2f}{'+' if float(c['uwz'])>0 else '-'}"
        else: key=fam
        g[key].append((r,c))
for k in sorted(g):
    v=g[k]
    d=[f(r,'stop_dist_max_m') for r,c in v]; t=[f(r,'stop_time_s') for r,c in v]; y=[f(r,'stop_yaw_deg') for r,c in v]
    sp=[abs(f(c,'steady_vx')) for r,c in v]; w=[abs(f(c,'steady_wz')) for r,c in v]
    print('%-24s n=%3d v<=%.3f w<=%.3f dmax=%.3f tmax=%.2f yaw=[%.1f,%.1f] held=%s'%(k,len(v),max(sp),max(w),max(d),max(t),min(y),max(y),all(r['settled_hold']=='True' for r,c in v)))
