if __name__ == "__main__":
    import os
    from pathlib import Path
    import sys
    import traceback
    try:
        from spot_controller.ui import main
        result = main()
    except Exception:
        path = Path(os.environ.get('LOCALAPPDATA', str(Path.home()))) / 'SpotOMGController' / 'error.log'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(traceback.format_exc(), encoding='utf-8')
        if sys.stderr is not None:
            print(path.read_text(encoding='utf-8'), file=sys.stderr)
        result = 1
    raise SystemExit(result)
