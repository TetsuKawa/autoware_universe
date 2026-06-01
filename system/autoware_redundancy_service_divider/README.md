# autoware_redundancy_service_divider

## Purpose

Provides a single node that integrates two complementary functions for a redundant (main/sub ECU) architecture:

1. **ServiceDivider** — receives external service requests and forwards them simultaneously to both main and sub.
2. **AutowareReadyDivider** — monitors Autoware initialization state and proactively issues reset/init requests to both main and sub at the appropriate timing.

---

## Inner-workings / Algorithms

### ServiceDivider

For each incoming service request:

1. Send the request to both main and sub clients simultaneously (`async_send_request`)
2. Wait for both responses up to `service_timeout_ms`
3. Return success only if both succeed; return the first failure's code and message otherwise

### AutowareReadyDivider

#### Startup initialization

At startup, a 1-second periodic timer runs an `InitState` state machine through the following steps.
Each step is retried on the next timer tick if a service call fails or the service is not yet available.

| State | Action |
|---|---|
| `WAIT_SERVICES_READY` | Poll all service pairs with `service_is_ready()` (non-blocking) |
| `SET_AGGREGATOR_INIT` | Call `set_aggregator_initializing(true)` on both main/sub |
| `RESET_SWITCHER` | Call `reset_redundancy_switcher()` on both main/sub |
| `SET_SWITCHER_IFACE_INIT` | Call `set_redundancy_switcher_interface_initializing(true)` on both main/sub |
| `DONE` | Cancel the init timer; start the 5-second periodic timer |

#### Normal operation

Once initialization is complete, a 5-second periodic timer fires while the system remains in initializing state:

- **Autoware not yet ready**: calls `reset_redundancy_switcher()` as a defensive periodic reset.
- **Autoware ready but service calls previously failed**: retries the full ready-state sequence below.

When all three conditions are met simultaneously (localization initialized, route set, Autoware control enabled), the node:

1. Calls `set_aggregator_initializing(false)` on both main/sub
2. Calls `reset_redundancy_switcher()` on both main/sub
3. Calls `set_redundancy_switcher_interface_initializing(false)` on both main/sub

---

## Inputs / Outputs

### [ServiceDivider] Input Services

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `~/input/change_operation_mode` | `autoware_system_msgs::srv::ChangeOperationMode` | Change operation mode request |
| `~/input/change_autoware_control` | `autoware_system_msgs::srv::ChangeAutowareControl` | Change autoware control request |
| `~/input/reset_mrm` | `tier4_external_api_msgs::srv::ResetMrm` | Triggers reset_diag_graph and reset_redundancy_switcher on all clients |

### [ServiceDivider] Output Service Clients (main / sub)

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `~/output/change_operation_mode/{main,sub}` | `autoware_system_msgs::srv::ChangeOperationMode` | Forwarded to main/sub system |
| `~/output/change_autoware_control/{main,sub}` | `autoware_system_msgs::srv::ChangeAutowareControl` | Forwarded to main/sub system |
| `~/output/reset_diag_graph/{main,sub}` | `tier4_system_msgs::srv::ResetDiagGraph` | Forwarded to main/sub system |
| `~/output/reset_redundancy_switcher/{main,sub}` | `tier4_system_msgs::srv::ResetRedundancySwitcher` | Forwarded to main/sub system (**shared** with AutowareReadyDivider) |

### [AutowareReadyDivider] Subscriptions

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `~/input/localization_initialization_state` | `autoware_adapi_v1_msgs::msg::LocalizationInitializationState` | Localization initialization state |
| `~/input/route_state` | `autoware_adapi_v1_msgs::msg::RouteState` | Route state |
| `~/input/operation_mode_state` | `autoware_adapi_v1_msgs::msg::OperationModeState` | Operation mode state |

### [AutowareReadyDivider] Output Service Clients (main / sub)

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `~/output/set_aggregator_initializing/{main,sub}` | `std_srvs::srv::SetBool` | Set diagnostic aggregator initialization state |
| `~/output/reset_redundancy_switcher/{main,sub}` | `tier4_system_msgs::srv::ResetRedundancySwitcher` | Reset redundancy switcher (**shared** with ServiceDivider) |
| `~/output/set_redundancy_switcher_interface_initializing/{main,sub}` | `std_srvs::srv::SetBool` | Set redundancy switcher interface initialization state |

---

## Parameters

| Name | Type | Default | Description |
| ---- | ---- | ------- | ----------- |
| `service_timeout_ms` | int | 200 | Timeout in milliseconds for each service call (applied to both ServiceDivider and AutowareReadyDivider) |
| `is_redundant` | bool | true | Whether the sub ECU is present. When `false`, all sub-side clients are skipped and `reset_redundancy_switcher` is not called — silently, as if they never existed |

### Behavior when `is_redundant: false`

| Feature | `is_redundant: true` | `is_redundant: false` |
|---|---|---|
| ServiceDivider `dispatch` | main + sub | main only |
| `on_reset_mrm` | reset_diag_graph × 2 + reset_redundancy_switcher × 2 | reset_diag_graph/main only |
| AutowareReadyDivider service calls | main + sub | main only |
| `reset_redundancy_switcher` | called on main + sub | not called |

---

## Response Status Codes

ServiceDivider responses use `autoware_common_msgs::msg::ResponseStatus`:
- Timeout → `SERVICE_TIMEOUT` (50002)
