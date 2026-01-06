import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ['esp32']

ble_mesh_gateway_ns = cg.esphome_ns.namespace('ble_mesh_gateway')
BleMeshGateway = ble_mesh_gateway_ns.class_('BleMeshGateway', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BleMeshGateway),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add_global(cg.RawStatement('#include "esphome/components/ble_mesh_gateway/ble_mesh_gateway.h"'))
