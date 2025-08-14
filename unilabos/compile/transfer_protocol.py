from unilabos.compile.pump_protocol import generate_pump_protocol_with_rinsing


def generate_transfer_protocol(graph, node, step_id):
    """
    Generate transfer protocol using pump protocol with default flow rates.
    This is a simplified version of PumpTransferProtocol for basic transfers.
    """
    # Add default flow rates for basic transfer protocol
    node_with_defaults = node.copy()
    
    # Set default flow rates if not present
    if not hasattr(node, 'flowrate'):
        node_with_defaults['flowrate'] = 2.5
    if not hasattr(node, 'transfer_flowrate'):
        node_with_defaults['transfer_flowrate'] = 0.5
    
    # Use the existing pump protocol generator
    return generate_pump_protocol_with_rinsing(graph, node_with_defaults, step_id)
