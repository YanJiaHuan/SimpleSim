import test from 'node:test';
import assert from 'node:assert/strict';

import { buildAccessoryRows, buildKeypadFlags } from './panel-model.js';

test('buildKeypadFlags hides switch-arm and linear-axis controls for single-arm robots', () => {
  const flags = buildKeypadFlags({
    controls: {
      can_switch_arm: false,
      show_linear_axis_controls: false,
    },
  });

  assert.equal(flags.showSwitchArm, false);
  assert.equal(flags.showLinearAxis, false);
});

test('buildAccessoryRows returns one row per accessory descriptor', () => {
  const rows = buildAccessoryRows(
    {
      panels: {
        accessories: [
          { id: 'hand', kind: 'gripper', label: 'Hand', arm: 'main' },
        ],
      },
    },
    {
      accessory: {
        hand: {
          kind: 'gripper',
          label: 'Hand',
          arm: 'main',
          is_closed: false,
        },
      },
    },
  );

  assert.deepEqual(rows, [
    { id: 'hand', term: 'Hand', value: 'open' },
  ]);
});
