"""Validate corpus; no policy execution. Exit 2 unless every certificate PASSes."""
import argparse
import json
import yaml
from .schema import load_json, safe_path, validate_corpus
from .scenarios import certify
from .artifacts import load_catalog,build_bundle,output_path,persist


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--suite',action='append',help='diagnostic suites; repository-relative')
    group.add_argument('--catalog',help='authoritative full corpus catalog')
    parser.add_argument('--witnesses',help='evaluator-only episode ID -> pose list JSON')
    parser.add_argument('--output',help='new repository-relative validation bundle directory')
    args = parser.parse_args()
    try:
        if args.catalog:
            if args.witnesses:raise ValueError('catalog already owns witness path')
            catalog,suites,configs,witnesses=load_catalog(args.catalog)
            files,results=build_bundle(catalog,suites,configs,witnesses)
            publication=persist(files,output_path(args.output)) if args.output else 'NOT_WRITTEN'
            print(json.dumps(dict(protocol='t1-0.2',certificates=results,publication=publication),indent=2))
            return 0 if all(r['status']=='PASS' for r in results) else 2
        if args.output:raise ValueError('artifact output requires a full corpus --catalog')
        suites = [load_json(safe_path(p)) for p in args.suite]
        configs = validate_corpus(suites)
        witnesses = load_json(safe_path(args.witnesses)) if args.witnesses else {}
        results = [dict(episode_id=e['episode_id'],**certify(e,s['shared'],c,witnesses.get(e['episode_id'])))
                   for s,c in zip(suites,configs) for e in s['episodes']]
        print(json.dumps({'protocol':'t1-0.2','certificates':results},ensure_ascii=False,allow_nan=False,indent=2))
        return 0 if all(r['status']=='PASS' for r in results) else 2
    except (ValueError,TypeError,KeyError,OSError,OverflowError,yaml.YAMLError) as error:
        print(json.dumps({'status':'FAIL','reason':str(error)},ensure_ascii=False))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
