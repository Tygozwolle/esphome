import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_NUMBER,
    CONF_MODE,
    CONF_INVERTED,
    CONF_OUTPUT,
)

CODEOWNERS = ["@Tygozwolle"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True
CONF_PIN_COUNT = "pin_count"
ch442g_ns = cg.esphome_ns.namespace("ch442g")

ch442gComponent = ch442g_ns.class_("ch442gComponent", cg.Component, i2c.I2CDevice)
ch442gGPIOPin = ch442g_ns.class_(
    "ch442gGPIOPin", cg.GPIOPin, cg.Parented.template(ch442gComponent)
)

CONF_ch442g = "ch442g"
CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.declare_id(ch442gComponent),
            cv.Optional(CONF_PIN_COUNT, default=8): cv.one_of(4, 8, 16),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(
        i2c.i2c_device_schema(0x26)
    )  # Note: 0x26 for the non-A part. The ch442g parts start at addess 0x38
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_pin_count(config[CONF_PIN_COUNT]))
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)


def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value


ch442g_PIN_SCHEMA = pins.gpio_base_schema(
    ch442gGPIOPin,
    cv.int_range(min=0, max=15),
    modes=[CONF_INPUT, CONF_OUTPUT],
    mode_validator=validate_mode,
).extend(
    {
        cv.Required(CONF_ch442g): cv.use_id(ch442gComponent),
    }
)


def ch442g_pin_final_validate(pin_config, parent_config):
    count = parent_config[CONF_PIN_COUNT]
    if pin_config[CONF_NUMBER] >= count:
        raise cv.Invalid(f"Pin number must be in range 0-{count - 1}")


@pins.PIN_SCHEMA_REGISTRY.register(
    CONF_ch442g, ch442g_PIN_SCHEMA, ch442g_pin_final_validate
)
async def ch442g_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_ch442g])

    cg.add(var.set_parent(parent))

    num = config[CONF_NUMBER]
    cg.add(var.set_pin(num))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var
