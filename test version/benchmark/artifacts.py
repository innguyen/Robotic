"""Corpus catalog, reproducible evidence, and atomic immutable validation bundles."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from .schema import ROOT, fields, identifier, load_json, safe_path, validate_corpus
from .scenarios import certify, resolved_episode_bytes


def encode(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf8')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def load_catalog(name,root=ROOT):
    catalog=load_json(safe_path(name,root))
    fields(catalog,('schema_version','suites','witnesses','families'))
    if catalog['schema_version']!='t1-catalog-1' or type(catalog['suites']) is not list or not catalog['suites']:
        raise ValueError('invalid corpus catalog')
    if len(set(catalog['suites']))!=len(catalog['suites']):raise ValueError('duplicate catalog path')
    suites=[load_json(safe_path(p,root)) for p in catalog['suites']]
    cfgs=validate_corpus(suites,root)
    witnesses=load_json(safe_path(catalog['witnesses'],root))
    ids={e['episode_id'] for s in suites for e in s['episodes']}
    if type(witnesses) is not dict or set(witnesses)-ids:raise ValueError('unknown witness episode')
    families={e['layout_family_id'] for s in suites for e in s['episodes']}
    if type(catalog['families']) is not dict or set(catalog['families'])!=families:raise ValueError('incomplete family provenance')
    for key,p in catalog['families'].items():
        identifier(key);fields(p,('source','reviewed','notes'))
        if p['reviewed'] is not True or any(type(p[k]) is not str or not p[k].strip() for k in ('source','notes')):
            raise ValueError('family provenance requires explicit manual review')
    return catalog,suites,cfgs,witnesses


def build_bundle(catalog,suites,cfgs,witnesses):
    files={}; results=[]
    verifier={p.name:digest(p.read_bytes()) for p in sorted(Path(__file__).parent.glob('*.py'))}
    for s,cfg in zip(suites,cfgs):
        for e in s['episodes']:
            evidence=witnesses.get(e['episode_id'])
            certificate=certify(e,s['shared'],cfg,evidence)
            certificate['verifier_hash']=digest(encode(verifier))
            prefix=e['episode_id']+'/'
            files[prefix+'resolved_episode.json']=resolved_episode_bytes(e,s['shared'],cfg)
            files[prefix+'evidence.json']=encode(evidence)
            files[prefix+'fixture_validation.json']=encode(certificate)
            results.append(dict(episode_id=e['episode_id'],status=certificate['status'],reason=certificate['reason']))
    manifest=dict(protocol='t1-0.2',catalog=catalog,verifier=verifier,
                  files={name:digest(data) for name,data in sorted(files.items())})
    files['manifest.json']=encode(manifest)
    return files,results


def output_path(value,root=ROOT):
    if type(value) is not str or ':' in value or '\\' in value or value.startswith('/') or any(p in ('','..','.') for p in value.split('/')):
        raise ValueError('unsafe output path')
    root=Path(root).resolve(); dest=root/value
    # Reject symlinks even if their current target remains inside the workspace.
    for p in (dest,*dest.parents):
        if p==root:break
        if p.is_symlink():raise ValueError('output symlink forbidden')
    if not dest.resolve().is_relative_to(root):raise ValueError('output escapes root')
    return dest


def persist(files,destination):
    """Same-parent staging + rename publishes the complete bundle, never half of it.
    Existing bundles are immutable; recomputation must match EVERY byte/file.
    """
    destination=Path(destination)
    if destination.exists():
        actual={str(p.relative_to(destination)).replace('\\','/'):p.read_bytes()
                for p in destination.rglob('*') if p.is_file() and not p.is_symlink()}
        if any(p.is_symlink() for p in destination.rglob('*')) or actual!=files:
            raise ValueError('stale/tampered/incomplete validation bundle; use new output')
        return 'VERIFIED_EXISTING'
    destination.parent.mkdir(parents=True,exist_ok=True)
    staging=Path(tempfile.mkdtemp(prefix='.t1-validation-',dir=destination.parent))
    try:
        for name,data in files.items():
            p=staging/name;p.parent.mkdir(parents=True,exist_ok=True)
            with p.open('xb') as handle:
                handle.write(data);handle.flush();os.fsync(handle.fileno())
        os.rename(staging,destination)
    finally:
        if staging.exists():shutil.rmtree(staging)
    return 'PUBLISHED'
