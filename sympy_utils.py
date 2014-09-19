import sympy
from numpy import f2py
import pprint
from sympy.utilities import codegen
import StringIO
import re

# require newer sympy for codegen
if not sympy.__version__.split('-')[0] >= '0.7.5':
        raise RuntimeError('install newer sympy version')


def rhs_to_scipy_ode(rhs, t, x_vect, u_vect,
                     constants, *args, **lambdify_kwargs):
    """Convert rhs to lambda function and jacobian comaptible with scipy.

    Given a state space description of a dynamic system, create
    a lambda function with a signature compatible with
    scipy.ode.integrate: f(t, x, u, *args).

    Parameters
    ----------
    rhs : sympy.Matrix
        Column vector of expressions for
        the derivative of x (continuous),
        or for the change in x (discrete).
    t : sympy.Symbol
        The independent time-like variable.
    x_vect : sympy.Matrix
        Column vector of symbols in the x (state) vector.
    u_vect : sympy.Matrix
        Column vector of symbols in the u (input) vector.
        Most controllers pass a vector of inputs and this
        supports this structure.
    constants: dict
        Dictionary of constants to substitute into rhs.
    *args: dict
        Additioinal symbols in f_vect that are not states, inputs
        or constants.
    **lambdify_kwargs:
        Additional arugments to pass to lambdify.
        @see sympy.utilities.lambdify.lambdify

    Returns
    -------
    f : function
        A lambda function that compute the right hand side
        of the ode: f(t, x, u, *f_args).
    jac : function
        A lambda function that computes the jacobian
        of the right hand side with respect to the state x:
        jac(t, x, u, *jac_args).
    """
    if constants is not None:
        rhs = rhs.subs(constants)
    x = sympy.DeferredVector('x')
    u = sympy.DeferredVector('u')
    ss_subs = {x_vect[i]: x[i] for i in range(len(x_vect))}
    ss_subs.update({u_vect[i]: u[i] for i in range(len(u_vect))})
    if 'default_array' not in lambdify_kwargs.keys():
        lambdify_kwargs['default_array'] = True
    f = sympy.lambdify((t, x, u) + args, rhs.subs(ss_subs),
                       **lambdify_kwargs)
    jac_vect = rhs.jacobian(x_vect)
    jac = sympy.lambdify(
        (t, x, u) + args, jac_vect.subs(ss_subs),
        **lambdify_kwargs)
    return (f, jac)


def save_sympy_expr(expr, filename):
    with open(filename, 'wb') as save_file:
        s = re.sub("(?P<key>'[^']*':)", '\n\g<key>', sympy.python(expr))
        save_file.write(s)


def load_sympy_expr(filename):
    with open(filename, 'r') as load_file:
        load_string = load_file.read()
    exec('from sympy import *')
    exec(load_string)
    e = locals()['e']
    return e


def save_repr(d, filename, env_string=''):
    with open(filename, 'wb') as save_file:
        save_file.write(env_string)
        save_file.write('e=' + pprint.saferepr(d))


def load_repr(filename):
    with open(filename, 'r') as load_file:
        load_string = load_file.read()
    exec(load_string)
    e = locals()['e']
    return e


def create_fortran_module_from_sympy_save(module, expr_file):
    source = gen_fortran_module(
        module=module,
        **load_sympy_expr(expr_file))
    compile_fortran_module(
        filename=module+'.f90',
        module=module,
        source=source)
    exec('import {:s}'.format(module))
    return locals()[module]


def compile_fortran_module(filename, module, source=None):
    if source is None:
        source = open(filename, 'r').read()
    if f2py.compile(source=source, modulename=module,
                    source_fn=filename):
        raise RuntimeError('compile failed, see console')


def gen_fortran_module(
        module, t, x, u, f, g_dict, const,
        project='PX4-SIL', header=True):
    x_ = sympy.MatrixSymbol('x', len(x), 1)
    u_ = sympy.MatrixSymbol('u', len(u), 1)
    ss_sub = {x[i]: x_[i, 0] for i in range(len(x))}
    ss_sub.update({u[i]: u_[i, 0] for i in range(len(u))})
    name_expr = [
        ('f', f),
        ('A', f.jacobian(x)),
        ('B', f.jacobian(u))]
    for key in g_dict.keys():
        name = key
        expr = g_dict[key]
        name_expr.append((name, expr))
        name_expr.append((name+'_H', expr.jacobian(x)))

    routines = []
    for name, expr in name_expr:
        out = sympy.MatrixSymbol('out', expr.shape[0], expr.shape[1])
        expr = expr.applyfunc(lambda e: e.simplify())
        routines.append(codegen.Routine(
            'compute_'+name,
            sympy.Equality(out, expr.subs(ss_sub)),
            (out, t, x_, u_) + const))

    fgen = codegen.FCodeGen(project)
    s = StringIO.StringIO()
    fgen.dump_f95(routines, s, module, header=header)
    src = s.getvalue()
    s.close()
    return src
