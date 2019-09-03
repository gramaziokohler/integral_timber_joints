# -*- coding: utf-8 -*-
from __future__ import print_function

import codecs
import contextlib
import glob
import os
import sys
from shutil import rmtree
from xml.dom.minidom import parse

from invoke import Collection, Exit, task

try:
    input = raw_input
except NameError:
    pass
BASE_FOLDER = os.path.dirname(__file__)


class Log(object):
    def __init__(self, out=sys.stdout, err=sys.stderr):
        self.out = out
        self.err = err

    def flush(self):
        self.out.flush()
        self.err.flush()

    def write(self, message):
        self.flush()
        self.out.write(message + '\n')
        self.out.flush()

    def info(self, message):
        self.write('[INFO] %s' % message)

    def warn(self, message):
        self.write('[WARN] %s' % message)


log = Log()


def confirm(question):
    while True:
        response = input(question).lower().strip()

        if not response or response in ('n', 'no'):
            return False

        if response in ('y', 'yes'):
            return True

        print('Focus, kid! It is either (y)es or (n)o', file=sys.stderr)


# The IronPython install code is based on gh_python_remote
# https://github.com/Digital-Structures/ghpythonremote
# MIT License
# Copyright (c) 2017 Pierre Cuvilliers, Caitlin Mueller, Massachusetts Institute of Technology
def get_ironpython_path(rhino_version):
    appdata_path = os.getenv('APPDATA', '')
    ironpython_settings_path = os.path.join(appdata_path, 'McNeel', 'Rhinoceros', rhino_version, 'Plug-ins',
                                            'IronPython (814d908a-e25c-493d-97e9-ee3861957f49)', 'settings')

    if not os.path.isdir(ironpython_settings_path):
        return None

    return ironpython_settings_path


def replaceText(node, newText):
    if node.firstChild.nodeType != node.TEXT_NODE:
        raise Exception("Node does not contain text")

    node.firstChild.replaceWholeText(newText)


def updateSearchPaths(settings_file, python_source_path):
    with codecs.open(settings_file, 'r', encoding="ascii", errors="ignore") as file_handle:
        doc = parse(file_handle)

    for entry in doc.getElementsByTagName('entry'):
        if entry.getAttribute('key') == 'SearchPaths':
            current_paths = entry.firstChild.data
            if python_source_path not in current_paths:
                replaceText(entry, current_paths + ';' + python_source_path)

    with codecs.open(settings_file, 'w', encoding='utf-8') as file_handle:
        doc.writexml(file_handle)


@task(default=True)
def help(ctx):
    """Lists available tasks and usage."""
    ctx.run('invoke --list')
    log.write('Use "invoke -h <taskname>" to get detailed help for a task.')


@task(help={
    'docs': 'True to generate documentation, otherwise False',
    'bytecode': 'True to clean up compiled python files, otherwise False.',
    'builds': 'True to clean up build/packaging artifacts, otherwise False.'})
def clean(ctx, docs=True, bytecode=True, builds=True):
    """Cleans the local copy from compiled artifacts."""
    if builds:
        ctx.run('python setup.py clean')

    if bytecode:
        for root, dirs, files in os.walk(BASE_FOLDER):
            for f in files:
                if f.endswith('.pyc'):
                    os.remove(os.path.join(root, f))
            if '.git' in dirs:
                dirs.remove('.git')

    folders = []

    if docs:
        folders.append('docs/_build/')
        folders.append('dist/')

    if bytecode:
        folders.append('src/integral_timber_joints/__pycache__')

    if builds:
        folders.append('build/')
        folders.append('src/integral_timber_joints.egg-info/')

    for folder in folders:
        rmtree(os.path.join(BASE_FOLDER, folder), ignore_errors=True)

@task(help={
      'rebuild': 'True to clean all previously built docs before starting, otherwise False.',
      'check_links': 'True to check all web links in docs for validity, otherwise False.'})
def docs(ctx, rebuild=True, check_links=False):
    """Builds package's HTML documentation."""
    if rebuild:
        clean(ctx)
    ctx.run('sphinx-build -b doctest docs dist/docs')
    ctx.run('sphinx-build -b html docs dist/docs')
    if check_links:
        ctx.run('sphinx-build -b linkcheck docs dist/docs')


@task()
def check(ctx):
    """Check the consistency of documentation, coding style and a few other things."""
    log.write('Checking ReStructuredText formatting...')
    ctx.run('python setup.py check --strict --metadata --restructuredtext')

    log.write('Running flake8 python linter...')
    ctx.run('flake8 src setup.py')

    log.write('Checking python imports...')
    ctx.run('isort --check-only --diff --recursive src tests setup.py')

    log.write('Checking MANIFEST.in...')
    ctx.run('check-manifest')


@task(help={
      'checks': 'True to run all checks before testing, otherwise False.'})
def test(ctx, checks=True):
    """Run all tests."""
    if checks:
        check(ctx)

    ctx.run('pytest --doctest-module')

@task(help={
      'release_type': 'Type of release follows semver rules. Must be one of: major, minor, patch.'})
def release(ctx, release_type):
    """Releases the project in one swift command!"""
    if release_type not in ('patch', 'minor', 'major'):
        raise Exit('The release type parameter is invalid.\nMust be one of: major, minor, patch')

    ctx.run('bumpversion %s --verbose' % release_type)
    ctx.run('invoke docs test')
    ctx.run('python setup.py clean --all sdist bdist_wheel')

    if confirm('You are about to upload the release to pypi.org. Are you sure? [y/N]'):
        files = ['dist/*.whl', 'dist/*.gz', 'dist/*.zip']
        dist_files = ' '.join([pattern for f in files for pattern in glob.glob(f)])

        if len(dist_files):
            ctx.run('twine upload --skip-existing %s' % dist_files)
        else:
            raise Exit('No files found to release')
    else:
        raise Exit('Aborted release')


@task()
def add_to_rhino(ctx):
    """Adds the current project to Rhino Python search paths."""
    try:
        python_source_path = os.path.join(os.getcwd(), 'src')
        rhino_setting_per_version = [
            ('5.0', 'settings.xml'), ('6.0', 'settings-Scheme__Default.xml')]
        setting_files_updated = 0

        for version, filename in rhino_setting_per_version:
            ironpython_path = get_ironpython_path(version)

            if not ironpython_path:
                continue

            settings_file = os.path.join(ironpython_path, filename)
            if not os.path.isfile(settings_file):
                log.warn('IronPython settings for Rhino ' + version + ' not found')
            else:
                updateSearchPaths(settings_file, python_source_path)
                log.write('Updated search path for Rhino ' + version)
                setting_files_updated += 1

        if setting_files_updated == 0:
            raise Exit('[ERROR] No Rhino settings file found\n' +
                       'Could not automatically make this project available to IronPython\n' +
                       'To add manually, open EditPythonScript on Rhinoceros, go to Tools -> Options\n' +
                       'and add the project path to the module search paths')

    except RuntimeError as error:
        raise Exit(error)


@contextlib.contextmanager
def chdir(dirname=None):
    current_dir = os.getcwd()
    try:
        if dirname is not None:
            os.chdir(dirname)
        yield
    finally:
        os.chdir(current_dir)
