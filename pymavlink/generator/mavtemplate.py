#!/usr/bin/env python
'''
simple templating system for mavlink generator

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''

from mavparse import MAVParseError

class MAVTemplate(object):
    '''simple templating system'''
    def __init__(self,
                 start_var_token="${", 
                 end_var_token="}", 
                 start_rep_token="${{", 
                 end_rep_token="}}",
                 trim_leading_lf=True,
                 checkmissing=True,
                 ros_types=False):
        self.start_var_token = start_var_token
        self.end_var_token = end_var_token
        self.start_rep_token = start_rep_token
        self.end_rep_token = end_rep_token
        self.trim_leading_lf = trim_leading_lf
        self.checkmissing = checkmissing
        self.ros_types = ros_types

    def find_end(self, text, start_token, end_token):
        '''find the of a token.
        Returns the offset in the string immediately after the matching end_token'''
        if not text.startswith(start_token):
            raise MAVParseError("invalid token start")
        offset = len(start_token)
        nesting = 1
        while nesting > 0:
            idx1 = text[offset:].find(start_token)
            idx2 = text[offset:].find(end_token)
            if idx1 == -1 and idx2 == -1:
                raise MAVParseError("token nesting error")
            if idx1 == -1 or idx1 > idx2:
                offset += idx2 + len(end_token)
                nesting -= 1
            else:
                offset += idx1 + len(start_token)
                nesting += 1
        return offset

    def find_var_end(self, text):
        '''find the of a variable'''
        return self.find_end(text, self.start_var_token, self.end_var_token)

    def find_rep_end(self, text):
        '''find the of a repitition'''
        return self.find_end(text, self.start_rep_token, self.end_rep_token)

    def substitute(self, text, subvars={},
                   trim_leading_lf=None, checkmissing=None, ros_types=None):
        '''substitute variables in a string'''

        if trim_leading_lf is None:
            trim_leading_lf = self.trim_leading_lf
        if checkmissing is None:
            checkmissing = self.checkmissing
        if ros_types is None:
            ros_types = self.ros_types

        # handle repititions
        while True:
            subidx = text.find(self.start_rep_token)
            if subidx == -1:
                break
            endidx = self.find_rep_end(text[subidx:])
            if endidx == -1:
                raise MAVParseError("missing end macro in %s" % text[subidx:])
            part1 = text[0:subidx]
            part2 = text[subidx+len(self.start_rep_token):subidx+(endidx-len(self.end_rep_token))]
            part3 = text[subidx+endidx:]
            a = part2.split(':')
            field_name = a[0]
            rest = ':'.join(a[1:])
            v = getattr(subvars, field_name, None)
            if v is None:
                raise MAVParseError('unable to find field %s' % field_name)
            t1 = part1
            for f in v:
                t1 += self.substitute(rest, f, trim_leading_lf=False, checkmissing=False)
            if len(v) != 0 and t1[-1] in ["\n", ","]:
                t1 = t1[:-1]
            t1 += part3
            text = t1
                
        if ros_types: 
            text = text.replace("int8_t", "int8") 
            text = text.replace("uint8_t", "uint8")
            text = text.replace("int16_t", "int16")    
            text = text.replace("uint16_t", "uint16")
            text = text.replace("int32_t", "int32")   
            text = text.replace("uint32_t", "uint32")
            text = text.replace("int64_t","int64")
            text = text.replace("float", "float32")
            text = text.replace("double", "float64")
                
        if trim_leading_lf:
            if text[0] == '\n':
                text = text[1:]
        while True:
            idx = text.find(self.start_var_token)
            if idx == -1:
                return text
            endidx = text[idx:].find(self.end_var_token)
            if endidx == -1:
                raise MAVParseError('missing end of variable: %s' % text[idx:idx+10])
            varname = text[idx+2:idx+endidx]
            if isinstance(subvars, dict):
                if not varname in subvars:
                    if checkmissing:
                        raise MAVParseError("unknown variable in '%s%s%s'" % (
                            self.start_var_token, varname, self.end_var_token))
                    return text[0:idx+endidx] + self.substitute(text[idx+endidx:], subvars,
                                                                trim_leading_lf=False, checkmissing=False)
                value = subvars[varname]
            else:
                value = getattr(subvars, varname, None)
                if value is None:
                    if checkmissing:
                        raise MAVParseError("unknown variable in '%s%s%s'" % (
                            self.start_var_token, varname, self.end_var_token))
                    return text[0:idx+endidx] + self.substitute(text[idx+endidx:], subvars,
                                                                trim_leading_lf=False, checkmissing=False)
            text = text.replace("%s%s%s" % (self.start_var_token, varname, self.end_var_token), str(value))
        return text

    def write(self, file, text, subvars={}, trim_leading_lf=True, ros_types=False):
        '''write to a file with variable substitution'''
        file.write(self.substitute(text, subvars=subvars, trim_leading_lf=trim_leading_lf, ros_types=ros_types))
