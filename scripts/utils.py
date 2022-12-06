import logging
from pathlib import Path
import yaml

formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')


# todo: in pycharm, set format pattern to
#   ^(\d{4}-\d{2}-\d{2}\s\d{2}:\d{2}:\d{2},\d{3})\s(\S*)\s*(\w*)\s*(.*)$


def setup_logger(name: str, log_file: Path, level=logging.INFO):
    print(f'logging [{name}] at [{log_file}]')

    log_file.parent.mkdir(exist_ok=True, parents=True)

    handler = logging.FileHandler(log_file)
    handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger


def read_yaml(p: Path):
    if not p.exists():
        raise FileExistsError(f'{p} does not exist')
    with p.open("r") as stream:
        return yaml.safe_load(stream)


def log_example():
    # first file logger
    logger = setup_logger('first_logger', Path('logs/first_logfile.log'))
    logger.info('This is just info message')

    # second file logger
    super_logger = setup_logger('second_logger', Path('logs/second_logfile.log'))
    super_logger.error('This is an error message')


if __name__ == '__main__':
    log_example()
