from compas.data import Data
from integral_timber_joints.process import RobotClampAssemblyProcess


class BeamReport(Data):
    def __init__(self, name=None):
        super(BeamReport, self).__init__(name=name)
        self._beam_ids = []
        self._info = {}  # type: dict[str, list[str]]
        self._warn = {}
        self._error = {}
        self.attributes = {}

    def add_info(self, beam_id, message):
        if beam_id not in self._beam_ids:
            self._beam_ids.append(beam_id)
        if beam_id not in self._info:
            self._info[beam_id] = []
        self._info[beam_id].append(message)

    def add_warn(self, beam_id, message):
        if beam_id not in self._beam_ids:
            self._beam_ids.append(beam_id)
        if beam_id not in self._warn:
            self._warn[beam_id] = []
        self._warn[beam_id].append(message)

    def add_error(self, beam_id, message):
        if beam_id not in self._beam_ids:
            self._beam_ids.append(beam_id)
        if beam_id not in self._error:
            self._error[beam_id] = []
        self._error[beam_id].append(message)

    def get_infos(self, beam_id):
        # type: (str) -> list(str)
        if beam_id not in self._info:
            return []
        else:
            return self._info[beam_id]

    def get_warns(self, beam_id):
        # type: (str) -> list(str)
        if beam_id not in self._warn:
            return []
        else:
            return self._warn[beam_id]

    def get_errors(self, beam_id):
        # type: (str) -> list(str)
        if beam_id not in self._error:
            return []
        else:
            return self._error[beam_id]

    def beams_with_warn_or_error(self):
        beam_ids = []
        return list(set(self._warn.keys() + self._error.keys()))

    @property
    def all_good(self):
        return self.beams_with_warn_or_error() == []

    @property
    def data(self):
        data = {}
        data['beam_ids'] = [self._beam_ids]
        data['info'] = [self._info]
        data['warn'] = [self._warn]
        data['error'] = [self._error]
        data['attributes'] = [self.attributes]
        return data

    @data.setter
    def data(self, data):
        self._beam_ids = data.get('beam_ids', [])
        self._info = data.get('info', {})
        self._warn = data.get('warn', {})
        self._error = data.get('error', {})
        self.attributes = data.get('attributes', {})

    def to_plain_text(self, process, info=False, warn=True, error=True):
        # type: (RobotClampAssemblyProcess, bool, bool, bool) -> list[str]
        text = []
        if self.all_good:
            text.append("No Errors or Warnings.")

        header_written = False
        for beam_id in self._beam_ids:
            def write(message):
                seq_n = process.assembly.sequence.index(beam_id)
                if not header_written:
                    text.append("Beam %s (seq_n=%i)" % (beam_id, seq_n))
                text.append(message)

            if info:
                for message in self.get_infos(beam_id):
                    write("- Info : %s" % message)
            if warn:
                for message in self.get_warns(beam_id):
                    write("- Warning : %s" % message)
            if error:
                for message in self.get_errors(beam_id):
                    write("- Error : %s" % message)
        return text
