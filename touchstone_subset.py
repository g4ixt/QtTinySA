#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  7 21:03:42 2021

This is a simplified subset of the code touchstone.py from scikit-rf, an open-source
Python package for RF and Microwave applications.

For Raspberry Pi without having to install the full scikit-rf, since this is incompatible with PiOS 'Buster'.

Used to import only s2p files in a suitable format from NanoVNA-F_V2 or similar hardware

"""

import numpy
import numpy as npy

class Touchstone:
    """
    Class to read touchstone s-parameter files.

    The reference for writing this class is the draft of the
    Touchstone(R) File Format Specification Rev 2.0 [#]_ and
    Touchstone(R) File Format Specification Version 2.0 [#]_

    References
    ----------
    .. [#] https://ibis.org/interconnect_wip/touchstone_spec2_draft.pdf
    .. [#] https://ibis.org/touchstone_ver2.0/touchstone_ver2_0.pdf
    """
    def __init__(self, file):
        """
        constructor

        Parameters
        ----------
        file : str or file-object
            touchstone file to load

        Examples
        --------
        From filename

        >>> t = rf.Touchstone('network.s2p')

        From file-object

        >>> file = open('network.s2p')
        >>> t = rf.Touchstone(file)
        """
        fid = get_fid(file)
        filename = fid.name
        ## file name of the touchstone data file
        self.filename = filename

        ## file format version.
        # Defined by default to 1.0, since version number can be omitted in V1.0 format
        self.version = '1.0'
        ## comments in the file header
        self.comments = None
        ## unit of the frequency (Hz, kHz, MHz, GHz)
        self.frequency_unit = None
        ## number of frequency points
        self.frequency_nb = None
        ## s-parameter type (S,Y,Z,G,H)
        self.parameter = None
        ## s-parameter format (MA, DB, RI)
        self.format = None
        ## reference resistance, global setup
        self.resistance = None
        ## reference impedance for each s-parameter
        self.reference = None

        ## numpy array of original s-parameter data
        self.sparameters = None
        ## numpy array of original noise data
        self.noise = None

        ## kind of s-parameter data (s1p, s2p, s3p, s4p)
        self.rank = None
        ## Store port names in a list if they exist in the file
        self.port_names = None

        self.comment_variables=None
        self.load_file(fid)

        self.gamma = []
        self.z0 = []

        # if self.is_from_hfss:
        #     self.get_gamma_z0_from_fid(fid)

        fid.close()

    def load_file(self, fid):
        """
        Load the touchstone file into the internal data structures.

        Parameters
        ----------
        fid : file object

        """

        filename=self.filename

        # Check the filename extension.
        # Should be .sNp for Touchstone format V1.0, and .ts for V2
        extension = filename.split('.')[-1].lower()

        if (extension[0] == 's') and (extension[-1] == 'p'): # sNp
            # check if N is a correct number
            try:
                self.rank = int(extension[1:-1])
            except (ValueError):
                raise (ValueError("filename does not have a s-parameter extension. It has  [%s] instead. please, correct the extension to of form: 'sNp', where N is any integer." %(extension)))
        elif extension == 'ts':
            pass
        else:
            raise Exception('Filename does not have the expected Touchstone extension (.sNp or .ts)')

        values = []
        while True:
            line = fid.readline()
            if not line:
                break
            # store comments if they precede the option line
            line = line.split('!',1)
            if len(line) == 2:
                if not self.parameter:
                    if self.comments == None:
                        self.comments = ''
                    self.comments = self.comments + line[1]
                elif line[1].startswith(' Port['):
                    try:
                        port_string, name = line[1].split('=', 1) #throws ValueError on unpack
                        name = name.strip()
                        garbage, index = port_string.strip().split('[', 1) #throws ValueError on unpack
                        index = int(index.rstrip(']')) #throws ValueError on not int-able
                        if index > self.rank or index <= 0:
                            print("Port name {0} provided for port number {1} but that's out of range for a file with extension s{2}p".format(name, index, self.rank))
                        else:
                            if self.port_names is None: #Initialize the array at the last minute
                                self.port_names = [''] * self.rank
                            self.port_names[index - 1] = name
                    except ValueError as e:
                        print("Error extracting port names from line: {0}".format(line))

            # remove the comment (if any) so rest of line can be processed.
            # touchstone files are case-insensitive
            line = line[0].strip().lower()

            # skip the line if there was nothing except comments
            if len(line) == 0:
                continue

            # grab the [version] string
            if line[:9] == '[version]':
                self.version = line.split()[1]
                continue

            # grab the [reference] string
            if line[:11] == '[reference]':
                # The reference impedances can be span after the keyword
                # or on the following line
                self.reference = [ float(r) for r in line.split()[2:] ]
                if not self.reference:
                    line = fid.readline()
                    self.reference = [ float(r) for r in line.split()]
                continue

            # grab the [Number of Ports] string
            if line[:17] == '[number of ports]':
                self.rank = int(line.split()[-1])
                continue

            # grab the [Number of Frequencies] string
            if line[:23] == '[number of frequencies]':
                self.frequency_nb = line.split()[-1]
                continue

            # skip the [Network Data] keyword
            if line[:14] == '[network data]':
                continue

            # skip the [End] keyword
            if line[:5] == '[end]':
                continue

            # the option line
            if line[0] == '#':
                toks = line[1:].strip().split()
                # fill the option line with the missing defaults
                toks.extend(['ghz', 's', 'ma', 'r', '50'][len(toks):])
                self.frequency_unit = toks[0]
                self.parameter = toks[1]
                self.format = toks[2]
                self.resistance = toks[4]
                if self.frequency_unit not in ['hz', 'khz', 'mhz', 'ghz']:
                    print('ERROR: illegal frequency_unit [%s]',  self.frequency_unit)
                    # TODO: Raise
                if self.parameter not in 'syzgh':
                    print('ERROR: illegal parameter value [%s]', self.parameter)
                    # TODO: Raise
                if self.format not in ['ma', 'db', 'ri']:
                    print('ERROR: illegal format value [%s]', self.format)
                    # TODO: Raise

                continue

            # collect all values without taking care of there meaning
            # we're separating them later
            values.extend([ float(v) for v in line.split() ])

        # let's do some post-processing to the read values
        # for s2p parameters there may be noise parameters in the value list
        values = numpy.asarray(values)
        if self.rank == 2:
            # the first frequency value that is smaller than the last one is the
            # indicator for the start of the noise section
            # each set of the s-parameter section is 9 values long
            pos = numpy.where(numpy.sign(numpy.diff(values[::9])) == -1)
            if len(pos[0]) != 0:
                # we have noise data in the values
                pos = pos[0][0] + 1   # add 1 because diff reduced it by 1
                noise_values = values[pos*9:]
                values = values[:pos*9]
                self.noise = noise_values.reshape((-1,5))

        if len(values)%(1+2*(self.rank)**2) != 0 :
            # incomplete data line / matrix found
            raise AssertionError

        # reshape the values to match the rank
        self.sparameters = values.reshape((-1, 1 + 2*self.rank**2))
        # multiplier from the frequency unit
        self.frequency_mult = {'hz':1.0, 'khz':1e3,
                               'mhz':1e6, 'ghz':1e9}.get(self.frequency_unit)
        # set the reference to the resistance value if no [reference] is provided
        if not self.reference:
            self.reference = [self.resistance] * self.rank

    def get_sparameter_names(self, format="ri"):
        """
        Generate a list of column names for the s-parameter data.
        The names are different for each format.

        Parameters
        ----------
        format : str
          Format: ri, ma, db, orig (where orig refers to one of the three others)

        Returns
        -------
        names : list
            list of strings

        """
        names = ['frequency']
        if format == 'orig':
            format = self.format
        ext1, ext2 = {'ri':('R','I'),'ma':('M','A'), 'db':('DB','A')}.get(format)
        for r1 in range(self.rank):
            for r2 in range(self.rank):
                names.append("S%i%i%s"%(r1+1,r2+1,ext1))
                names.append("S%i%i%s"%(r1+1,r2+1,ext2))
        return names

    def get_sparameter_data(self, format='ri'):
        """
        Get the data of the s-parameter with the given format.

        Parameters
        ----------
        format : str
          Format: ri, ma, db, orig

        supported formats are:
          orig:  unmodified s-parameter data
          ri:    data in real/imaginary
          ma:    data in magnitude and angle (degree)
          db:    data in log magnitude and angle (degree)

        Returns
        -------
        ret: list
            list of numpy.arrays

        """
        ret = {}
        if format == 'orig':
            values = self.sparameters
        else:
            values = self.sparameters.copy()
            # use frequency in hz unit
            values[:,0] = values[:,0]*self.frequency_mult
            if (self.format == 'db') and (format == 'ma'):
                values[:,1::2] = 10**(values[:,1::2]/20.0)
            elif (self.format == 'db') and (format == 'ri'):
                v_complex = ((10**values[:,1::2]/20.0)
                             * numpy.exp(1j*numpy.pi/180 * values[:,2::2]))
                values[:,1::2] = numpy.real(v_complex)
                values[:,2::2] = numpy.imag(v_complex)
            elif (self.format == 'ma') and (format == 'db'):
                values[:,1::2] = 20*numpy.log10(values[:,1::2])
            elif (self.format == 'ma') and (format == 'ri'):
                v_complex = (values[:,1::2] * numpy.exp(1j*numpy.pi/180 * values[:,2::2]))
                values[:,1::2] = numpy.real(v_complex)
                values[:,2::2] = numpy.imag(v_complex)
            elif (self.format == 'ri') and (format == 'ma'):
                v_complex = numpy.absolute(values[:,1::2] + 1j* self.sparameters[:,2::2])
                values[:,1::2] = numpy.absolute(v_complex)
                values[:,2::2] = numpy.angle(v_complex)*(180/numpy.pi)
            elif (self.format == 'ri') and (format == 'db'):
                v_complex = numpy.absolute(values[:,1::2] + 1j* self.sparameters[:,2::2])
                values[:,1::2] = 20*numpy.log10(numpy.absolute(v_complex))
                values[:,2::2] = numpy.angle(v_complex)*(180/numpy.pi)

        for i,n in enumerate(self.get_sparameter_names(format=format)):
            ret[n] = values[:,i]

        # transpose Touchstone V1 2-port files (.2p), as the order is (11) (21) (12) (22)
        file_name_ending = self.filename.split('.')[-1].lower()
        if self.rank == 2 and file_name_ending == "s2p":
            swaps = [ k for k in ret if '21' in k]
            for s in swaps:
                true_s = s.replace('21', '12')
                ret[s], ret[true_s] = ret[true_s], ret[s]

        return ret

        """
        Extracts Z0 and Gamma comments from fid.

        Parameters
        ----------
        fid : file object
        """
        gamma = []
        z0 = []
        def line2ComplexVector(s):
            return mf.scalar2Complex(npy.array([k for k in s.strip().split(' ')
                                                if k != ''][self.rank*-2:],
                                                dtype='float'))
        fid.seek(0)
        while True:
            line = fid.readline()
            if not line:
                break
            line = line.replace('\t', ' ')

            # HFSS adds gamma and z0 data in .sNp files using comments.
            # NB : each line(s) describe gamma and z0.
            #  But, depending on the HFSS version, either:
            #  - up to 4 ports only.
                #  - up to 4 ports only.
            #        for N > 4, gamma and z0 are given by additional lines
            #  - all gamma and z0 are given on a single line (since 2020R2)
            # In addition, some spurious '!' can remain in these lines
            if '! Gamma' in line:
                _line = line.replace('! Gamma', '').replace('!', '').rstrip()

                # check how many elements are in the first line
                nb_elem = len(_line.split())

                if nb_elem == 2*self.rank:
                    # case of all data in a single line
                    gamma.append(line2ComplexVector(_line.replace('!', '').rstrip()))
                else:
                    # case of Nport > 4 *and* data on additional multiple lines
                    for _ in range(int(npy.ceil(self.rank/4.0)) - 1):
                        _line += fid.readline().replace('!', '').rstrip()
                    gamma.append(line2ComplexVector(_line))


            if '! Port Impedance' in line:
                _line = line.replace('! Port Impedance', '').rstrip()
                nb_elem = len(_line.split())

                if nb_elem == 2*self.rank:
                    z0.append(line2ComplexVector(_line.replace('!', '').rstrip()))
                else:
                    for _ in range(int(npy.ceil(self.rank/4.0)) - 1):
                        _line += fid.readline().replace('!', '').rstrip()
                    z0.append(line2ComplexVector(_line))

        # If the file does not contain valid port impedance comments, set to default one
        if len(z0) == 0:
            z0 = npy.complex(self.resistance)
            #raise ValueError('Touchstone does not contain valid gamma, port impedance comments')

        self.gamma = npy.array(gamma)
        self.z0 = npy.array(z0)
def get_fid(file, *args, **kwargs):
    '''
    Return a file object, given a filename or file object.
    Useful when you want to allow the arguments of a function to
    be either files or filenames
    Parameters
    ----------
    file : str/unicode or file-object
        file to open
    \*args, \*\*kwargs : arguments and keyword arguments to `open()`
    Returns
    -------
    fid : file object
    '''
    if isinstance(file, str):
        return open(file, *args, **kwargs)
    else:
        return file

