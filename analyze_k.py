import json

rust = json.load(open('keep_rust_k_Channel.json'))
py = json.load(open('python_global_Channel_200x75.json'))

rm = {(r, c): v for r, c, v in zip(rust['K']['row'], rust['K']['col'], rust['K']['data'])}
pm = {(r, c): v for r, c, v in zip(py['K']['row'], py['K']['col'], py['K']['data'])}

print('F rust[:5]:', [round(x, 6) for x in rust['F'][:5]])
print('F py  [:5]:', [round(x, 6) for x in py['F'][:5]])
print()
print('C rust[:5]:', [round(x, 6) for x in rust['C'][:5]])
print('C py  [:5]:', [round(x, 6) for x in py['C'][:5]])
print()
print('DIAG K comparisons (first 10):')
for k in sorted(rm):
    if k[0] == k[1] and k[0] < 10:
        rv, pv = rm[k], pm[k]
        ratio = (rv / pv) if abs(pv) > 1e-12 else float('nan')
        print(f'  K[{k[0]},{k[1]}] rust={rv:+.6e} py={pv:+.6e} sum={rv+pv:+.3e} ratio={ratio:+.3f}')
print()
print('OFF-DIAG K comparisons (first 10):')
n = 0
for k in sorted(rm):
    if k[0] != k[1] and abs(pm[k]) > 1e-9:
        rv, pv = rm[k], pm[k]
        ratio = (rv / pv) if abs(pv) > 1e-12 else float('nan')
        print(f'  K{k} rust={rv:+.6e} py={pv:+.6e} sum={rv+pv:+.3e} ratio={ratio:+.3f}')
        n += 1
        if n >= 10:
            break
