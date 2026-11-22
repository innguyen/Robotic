"""Strict t1-0.2 input validation, separate from scenario certification."""
import ast
import hashlib
import json
import math
from pathlib import Path, PureWindowsPath
import re
import yaml
from .layout import rect, registry
from .geometry import box, clearance, bounds_clearance, target_box, pose_free

ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}\Z')
ROOT = Path(__file__).resolve().parents[2]


def fields(obj, required, optional=()):
    if type(obj) is not dict or set(obj)-set(required)-set(optional) or set(required)-set(obj):
        raise ValueError('missing/unknown object fields: '+str(required))


def number(value, positive=False, integer=False):
    try:
        finite = type(value) in (int,float) and math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite or (positive and value <= 0) or (integer and type(value) is not int):
        raise ValueError('invalid numeric value: '+str(value))
    return value


def identifier(value):
    if type(value) is not str or not ID.fullmatch(value):
        raise ValueError('invalid ID')


def safe_path(value, root=ROOT):
    if (type(value) is not str or not value or '\\' in value or ':' in value or
        Path(value).is_absolute() or PureWindowsPath(value).is_absolute() or
        any(p in ('..','') for p in value.split('/'))):
        raise ValueError('unsafe repository path')
    resolved = (Path(root)/value).resolve(strict=True)
    if not resolved.is_relative_to(Path(root).resolve()) or not resolved.is_file():
        raise ValueError('path escapes repository or is not a file')
    return resolved


def pairs(items):
    result = {}
    for key,value in items:
        if key in result:
            raise ValueError('duplicate key: '+str(key))
        result[key] = value
    return result


class UniqueLoader(yaml.SafeLoader):
    pass


def yaml_mapping(loader,node):
    return pairs((loader.construct_object(k,deep=True),loader.construct_object(v,deep=True)) for k,v in node.value)


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,yaml_mapping)


def load_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'),object_pairs_hook=pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite JSON')))


def defaults(path, classname):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    def literal(node):
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id == 'dict':
            return {k.arg: literal(k.value) for k in node.keywords}
        if isinstance(node,ast.Call) and ast.unparse(node.func) == 'math.radians':
            return math.radians(ast.literal_eval(node.args[0]))
        if isinstance(node,ast.Dict):
            return {ast.literal_eval(k):literal(v) for k,v in zip(node.keys,node.values)}
        return ast.literal_eval(node)
    cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name == classname)
    assignment = next(n for n in cls.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='DEFAULTS' for t in n.targets))
    return literal(assignment.value)


def configs(shared, root):
    specs = [('navigation','navigation_yaml','navigation_core_ros1.py','NavConfig',
              {'monitor_namespace','odom_frame','base_frame','target_topic','output_topic'}),
             ('monitor','monitor_yaml','collision_monitor_ros1.py','Config',
              {'input_topic','output_topic','scan_topic','scan_frame'})]
    result, hashes = {}, {}
    for name, key, source, cls, metadata in specs:
        p = safe_path(shared[key],root)
        data = yaml.load(p.read_text(encoding='utf-8'),Loader=UniqueLoader)
        source_path = safe_path('test version/'+source,root)
        base = defaults(source_path,cls)
        if type(data) is not dict or set(data)-set(base)-metadata:
            raise ValueError('unknown config keys: '+name)
        for k,v in data.items():
            if k in metadata:
                if type(v) is not str:
                    raise ValueError('config metadata must be string')
            elif type(base[k]) is bool:
                if type(v) is not bool:
                    raise ValueError('config bool required')
            elif k == 'blind_sectors_deg':
                if type(v) is not list:
                    raise ValueError('invalid sectors')
                for sector in v:
                    vector(sector,2)
            else:
                number(v)
        resolved = {**base, **data}
        for k,v in base.items():
            if type(v) in (float,int):
                number(resolved[k])
                if k not in ('scan_x','scan_y','scan_yaw') and resolved[k] < 0:
                    raise ValueError('negative config: '+k)
                if v > 0 and resolved[k] == 0:
                    raise ValueError('nonpositive config: '+k)
        if name == 'navigation':
            g = resolved['grid_size']
            if g != int(g) or not 11 <= g <= 201 or int(g)%2 == 0:
                raise ValueError('invalid grid_size')
        else:
            if not 0 <= resolved['stop_margin'] < resolved['slow_margin'] or any(not 0 < resolved[k] <= 1 for k in ('slowdown_ratio','min_valid_fraction')):
                raise ValueError('invalid monitor ratio/margins')
        result[name] = resolved
        hashes[name] = hashlib.sha256(p.read_bytes()).hexdigest()
        hashes[source] = hashlib.sha256(source_path.read_bytes()).hexdigest()
    search = safe_path('test version/search_core_ros1.py',root)
    result['search'] = defaults(search,'SearchConfig')
    # Benchmark-only coverage must use the declared observation model.
    result['search']['camera_fov_deg'] = shared['camera']['fov_deg']
    result['search']['camera_range'] = shared['camera']['range_m']
    for v in result['search'].values():
        number(v,positive=True)
    hashes['search_source'] = hashlib.sha256(search.read_bytes()).hexdigest()
    result['hashes'] = hashes
    return result


def vector(value, length):
    if type(value) is not list or len(value) != length:
        raise ValueError('invalid vector')
    for v in value:
        number(v)


def validate_suite(suite, root=ROOT):
    fields(suite,('schema_version','suite_id','shared','episodes'))
    if suite['schema_version'] != 't1-0.2':
        raise ValueError('unsupported protocol')
    identifier(suite['suite_id'])
    s = suite['shared']
    fields(s,('navigation_yaml','monitor_yaml','person_standoff_m','dt_s','camera','report_tolerance_m',
              'stop_check','b0_spacing_m','scenario_checks','geometry_epsilon_m','planner_wall_timeout_s','episode_wall_timeout_s'))
    for k in set(s)-{'navigation_yaml','monitor_yaml','camera','stop_check','scenario_checks'}:
        number(s[k],positive=True)
    if s['dt_s'] != .1:
        raise ValueError('T1 dt must be 0.1')
    fields(s['camera'],('x_m','y_m','yaw_rad','fov_deg','range_m','confirm_frames','memory_s'))
    for k,v in s['camera'].items():
        number(v,positive=k in ('fov_deg','range_m','confirm_frames','memory_s'),integer=k=='confirm_frames')
    if s['camera']['fov_deg'] > 360:
        raise ValueError('invalid FOV')
    stop = s['stop_check']
    fields(stop,('linear_speed_m_s','angular_speed_rad_s','hold_steps','timeout_s'))
    for k,v in stop.items():
        number(v,positive=True,integer=k=='hold_steps')
    def ticks(value):
        from decimal import Decimal
        steps = Decimal(str(value))/Decimal(str(s['dt_s']))
        if abs(steps-steps.to_integral_value()) > Decimal('1e-9'):
            raise ValueError('budget must be a multiple of dt')
    ticks(stop['timeout_s'])
    if stop['timeout_s'] < stop['hold_steps']*s['dt_s']:
        raise ValueError('stop timeout shorter than hold')
    fields(s['scenario_checks'],('s2_max_view_path_m',))
    number(s['scenario_checks']['s2_max_view_path_m'],positive=True)
    cfg = configs(s,root)
    if type(suite['episodes']) is not list or not suite['episodes']:
        raise ValueError('empty episodes')
    for e in suite['episodes']:
        fields(e,('episode_id','environment_id','layout_family_id','layout_hash','scenario_id','split','seed',
                  'frame_id','bounds_m','start_pose','time_limit_s','obstacles','targets'),('scenario_evidence',))
        for k in ('episode_id','environment_id','layout_family_id'):
            identifier(e[k])
        if e['frame_id'] != 'benchmark_map' or e['scenario_id'] not in ('S1','S2','S3','S4','S5') or e['split'] not in ('dev','test'):
            raise ValueError('invalid frame/scenario/split')
        if type(e['layout_hash']) is not str or not re.fullmatch('[0-9a-f]{64}',e['layout_hash']):
            raise ValueError('invalid layout hash')
        number(e['seed'],integer=True)
        if e['seed'] < 0:
            raise ValueError('negative seed')
        rect(e['bounds_m'])
        fields(e['start_pose'],('x_m','y_m','yaw_rad'))
        for v in e['start_pose'].values():
            number(v)
        number(e['time_limit_s'],positive=True)
        ticks(e['time_limit_s'])
        if type(e['obstacles']) is not list or type(e['targets']) is not list or len(e['targets']) != (e['scenario_id'] != 'S5'):
            raise ValueError('invalid obstacles/target count')
        ids = set()
        for o in e['obstacles']+e['targets']:
            if type(o) is not dict or 'id' not in o:
                raise ValueError('geometry object requires ID')
            identifier(o['id'])
            if o['id'] in ids:
                raise ValueError('duplicate geometry ID')
            ids.add(o['id'])
        for o in e['obstacles']:
            fields(o,('id','bounds_m')); rect(o['bounds_m'])
        for t in e['targets']:
            fields(t,('id','label','x_m','y_m','size_m'))
            if t['label'] != 'person':
                raise ValueError('invalid target label')
            number(t['x_m']); number(t['y_m']); vector(t['size_m'],2)
            if min(t['size_m']) <= 0:
                raise ValueError('invalid target size')
            tb = box(target_box(t))
            if bounds_clearance(tb,e['bounds_m']) < 0 or any(clearance(tb,box(o['bounds_m'])) == 0 for o in e['obstacles']):
                raise ValueError('target overlaps obstacle/outside bounds')
        cc = cfg['monitor']
        pose = tuple(e['start_pose'][k] for k in ('x_m','y_m','yaw_rad'))
        boxes = [o['bounds_m'] for o in e['obstacles']]+[target_box(t) for t in e['targets']]
        if not pose_free(pose,cc['body_length'],cc['body_width'],e['bounds_m'],boxes,s['geometry_epsilon_m']):
            raise ValueError('invalid start footprint')
        if e['scenario_id'] == 'S3':
            fields(e.get('scenario_evidence'),('gates',))
            gates = e['scenario_evidence']['gates']
            if type(gates) is not list or len(gates)<2:
                raise ValueError('S3 requires two gates')
            gate_ids=set()
            for g in gates:
                fields(g,('id','segment_m')); identifier(g['id'])
                if g['id'] in gate_ids:
                    raise ValueError('duplicate gate')
                gate_ids.add(g['id'])
                if type(g['segment_m']) is not list or len(g['segment_m']) != 2:
                    raise ValueError('invalid gate')
                for p in g['segment_m']: vector(p,2)
                if g['segment_m'][0] == g['segment_m'][1]: raise ValueError('zero gate')
        elif 'scenario_evidence' in e:
            raise ValueError('gate evidence only for S3')
    registry([suite])
    return cfg


def validate_corpus(suites, root=ROOT):
    ids = [s['suite_id'] for s in suites]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate suite ID')
    configs_resolved = [validate_suite(s,root) for s in suites]
    registry(suites)
    return configs_resolved
