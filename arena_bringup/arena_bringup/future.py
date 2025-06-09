
import collections.abc
import importlib
from typing import Sequence, Text

from launch import LaunchContext, SomeSubstitutionsType, Substitution
from launch.utilities import (ensure_argument_type,
                              normalize_to_list_of_substitutions,
                              perform_substitutions)
from launch.utilities.type_utils import perform_typed_substitution


# https://github.com/ros2/launch/blob/09403fcebedb68dd259e9ab6b35bd8712b6596ec/launch/launch/substitutions/python_expression.py
class PythonExpression(Substitution):
    """
    Substitution that can access contextual local variables.

    The expression may contain Substitutions, but must return something that can
    be converted to a string with `str()`.
    It also may contain math symbols and functions.
    """

    def __init__(self, expression: SomeSubstitutionsType,
                 python_modules: SomeSubstitutionsType = ['math']) -> None:
        """Create a PythonExpression substitution."""
        super().__init__()

        ensure_argument_type(
            expression,
            (str, Substitution, collections.abc.Iterable),
            'expression',
            'PythonExpression')

        ensure_argument_type(
            python_modules,
            (str, Substitution, collections.abc.Iterable),
            'python_modules',
            'PythonExpression')

        self.__expression = normalize_to_list_of_substitutions(expression)
        self.__python_modules = normalize_to_list_of_substitutions(python_modules)

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        """Parse `PythonExpression` substitution."""
        if len(data) < 1 or len(data) > 2:
            raise TypeError('eval substitution expects 1 or 2 arguments')
        kwargs = {}
        kwargs['expression'] = data[0]
        if len(data) == 2:
            # We get a text substitution from XML,
            # whose contents are comma-separated module names
            kwargs['python_modules'] = []
            # Check if we got empty list from XML
            # Ensure that we got a list!
            assert not isinstance(data[1], str)
            assert not isinstance(data[1], Substitution)
            # Modules
            modules = list(data[1])
            if len(modules) > 0:
                # XXX: What is going on here: the type annotation says we should get
                # a either strings or substitutions, but this says that we're
                # getting a substitution always?
                # Moreover, `perform` is called with `None`, which is not acceptable
                # for any substitution as far as I know (should be an empty launch context?)
                modules_str = modules[0].perform(None)  # type: ignore
                kwargs['python_modules'] = [module.strip() for module in modules_str.split(',')]
        return cls, kwargs

    @property
    def expression(self) -> list[Substitution]:
        """Getter for expression."""
        return self.__expression

    @property
    def python_modules(self) -> list[Substitution]:
        """Getter for python modules."""
        return self.__python_modules

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'PythonExpr({}, [{}])'.format(
            ' + '.join([sub.describe() for sub in self.expression]),
            ', '.join([sub.describe() for sub in self.python_modules]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by evaluating the expression."""
        module_names = [context.perform_substitution(sub) for sub in self.python_modules]
        module_objects = [importlib.import_module(name) for name in module_names]
        expression_locals = {}
        for module in module_objects:
            # For backwards compatibility, we allow math definitions to be implicitly
            # referenced in expressions, without prepending the math module name
            # TODO: This may be removed in a future release.
            if module.__name__ == 'math':
                expression_locals.update(vars(module))

            expression_locals[module.__name__] = module
        return str(eval(perform_substitutions(context, self.expression), {}, expression_locals))


class IfElseSubstitution(Substitution):
    def __init__(
        self,
        condition,
        if_value: SomeSubstitutionsType = '',
        else_value: SomeSubstitutionsType = ''
    ):
        super().__init__()
        if if_value == else_value == '':
            raise RuntimeError('One of if_value and else_value must be specified')
        self._condition = normalize_to_list_of_substitutions(condition)
        self._if_value = normalize_to_list_of_substitutions(if_value)
        self._else_value = normalize_to_list_of_substitutions(else_value)

    @property
    def condition(self):
        return self._condition

    @property
    def if_value(self):
        return self._if_value

    @property
    def else_value(self):
        return self._else_value

    def perform(self, context):
        try:
            condition = perform_typed_substitution(context, self.condition, bool)
        except (TypeError, ValueError) as e:
            raise e

        if condition:
            return perform_substitutions(context, self.if_value)
        else:
            return perform_substitutions(context, self.else_value)
