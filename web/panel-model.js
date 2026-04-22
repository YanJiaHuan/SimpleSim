export function buildKeypadFlags(meta) {
  const controls = meta?.controls ?? {};
  return {
    showSwitchArm: controls.can_switch_arm === true,
    showLinearAxis: controls.show_linear_axis_controls === true,
  };
}

export function buildAccessoryRows(meta, state) {
  const descriptors = meta?.panels?.accessories ?? [];
  const accessoryState = state?.accessory ?? {};

  return descriptors.map(item => {
    const current = accessoryState[item.id] ?? {};
    if (item.kind === 'linear_axis') {
      return {
        id: item.id,
        term: item.label,
        value: `${Number(current.value ?? 0).toFixed(3)} m`,
      };
    }
    if (item.kind === 'gripper') {
      return {
        id: item.id,
        term: item.label,
        value: current.is_closed ? 'closed' : 'open',
      };
    }
    return {
      id: item.id,
      term: item.label,
      value: '-',
    };
  });
}
