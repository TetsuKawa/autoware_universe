# Terminology and Data Flow Definition

このドキュメントは、Redundancy Switcher Interface で使用される重要な用語と、各データ構造の意未を定義します。

## 1. 主要な用語定義

### 1.1 Request（リクエスト）

**定義**: Switcher（上位系）から Core Logic に対して送信される一連の情報をひとまとめにしたもの

**構成ファイル**: `SwitcherRequestIR`

```cpp
SwitcherRequestIR {
  CommandIR command;           // コマンド内容
  std::string request_type;    // リクエストの種別（"switch", "status_query" など）
}
```

**含まれるもの**:

- `request_type`: リクエストの意図を示すカテゴリー
- `command`: リクエストに含まれる具体的な操作指示

**用途**: Switcher からの入力を一つの単位として受け取るための外側の枠組み

---

### 1.2 Command（コマンド）

**定義**: リクエストに含まれる具体的な操作指示

**構成ファイル**: `CommandIR`

```cpp
CommandIR {
  int command_id;              // コマンドの一意識別子
  std::string mode_request;    // 要求モード（"active", "standby", "inactive" など）
  int priority;                // 優先度
}
```

**含まれるもの**:

- `command_id`: 追跡・確認用の識別番号
- `mode_request`: 実際に何をさせたいのか（モード指定）
- `priority`: 複数のコマンドがある場合の優先順位

**用途**: Switcher の具体的な目的や指示を表現。複数のリクエストタイプに共有される情報

---

### 1.3 Status（ステータス）

**定義**: Core Logic から Switcher（上位系）に送信される現在のシステム状態

**構成ファイル**: `SystemStatusIR`

```cpp
SystemStatusIR {
  std::string current_mode;     // 現在のモード
  bool autoware_ok;             // Autoware が正常か
  bool is_switchable;           // 切り替え可能か
  int last_processed_id;        // 最後に処理したコマンドID
  std::string status_message;   // 詳細メッセージ
}
```

**含まれるもの**:

- 現在の動作モード
- システムの健全性
- 操作の可能性

**用途**: Switcher が現在のシステム状態を把握するための情報源。リクエストへの応答、定期的な状態通知

---

### 1.4 Control（制御）

**定義**: Core Logic から Autoware（下位系）に送信される介入・制御指示

**構成ファイル**: `ControlCommandIR`

```cpp
ControlCommandIR {
  std::string control_type;     // 制御の種類（"override", "monitor", "handover" など）
  std::string target_system;    // 対象システム
  std::string payload;          // 制御の詳細データ
}
```

**含まれるもの**:

- 何をするのか（override, monitor など）
- どのシステムに対して行うのか
- 具体的な制御パラメータ

**用途**: Autoware の動作に直接影響を与える指示。Mode 変更や Emergency 対応など

---

## 2. データフロー関係図

```text
┌─────────────┐
│   Switcher  │ (上位系)
│ (Upper)     │
└──────┬──────┘
       │
       ├─ SwitcherRequestIR ──────→  [Core Logic]
       │  (request_type, command_id, mode_request)
       │
       ← SystemStatusIR ───────────  Core Logic
         (current_mode, autoware_ok, is_switchable, last_processed_id)

┌─────────────┐
│ Autoware    │ (下位系)
│ (Lower)     │
└──────┬──────┘
       │
       ├─ AutowareInfoIR ────────→  [Core Logic]
       │  (vehicle_state, diagnostic_status)
       │
       ← ControlCommandIR ──────────  Core Logic
         (control_type, target_system, payload)
    or ← SystemStatusIR (monitoring) ─ Core Logic
```

---

## 3. 用語別の使い分け

| 用語                         | 送信元         | 送信先                | 目的                       |
| ---------------------------- | -------------- | --------------------- | -------------------------- |
| **Request**                  | Switcher       | Core Logic            | 操作依頼（Command を含む） |
| **Command**                  | （Request 内） | Core Logic の判定対象 | 具体的な操作内容           |
| **Status**                   | Core Logic     | Switcher              | 現在の状態報告             |
| **Control** (ControlCommand) | Core Logic     | Autoware              | 介入・制御指示             |

---

## 4. なぜこのような構造か

### Request が Command を包含する設計理由

1. **拡張性**: リクエストタイプ（"switch", "status_query" など）により、同じコマンド構造で異なる意図を表現可能
2. **追跡性**: request_type と command_id の両方により、複雑な状況下でも処理フローを追跡可能
3. **処理分岐**: Processor が request_type に応じて異なる処理を実行（例：status_query の場合は state 判定をスキップ、など）

### ControlCommand と SystemStatus の分離理由

1. **責務分離**: Control は "コマンドライク"（変化を引き起こす）、Status は "読み取り専用"（情報提供）
2. **安全性**: 制御指示と状態報告を分けることで、監視モードのみの機能が可能
3. **柔軟性**: Autoware への指示内容（control_type）は、Switcher への報告内容（status）と異なる可能性がある

---

## 5. 利用例

### 例1: 通常の切り替え要求

```cpp
Switcher → SwitcherRequestIR {
  request_type = "switch",
  command = CommandIR {
    command_id = 100,
    mode_request = "active",
    priority = 1
  }
}

↓ Processor が判定

Core Logic → Switcher: SystemStatusIR {
  current_mode = "active",
  autoware_ok = true,
  is_switchable = true,
  last_processed_id = 100
}

Core Logic → Autoware: ControlCommandIR {
  control_type = "handover",
  target_system = "planning_module",
  payload = "..."
}
```

### 例2: ステータス問い合わせ

```cpp
Switcher → SwitcherRequestIR {
  request_type = "status_query",
  command = CommandIR {
    command_id = 101,
    mode_request = "",
    priority = 0
  }
}

↓ Processor が判定（この場合、state 遷移は起こさない）

Core Logic → Switcher: SystemStatusIR {
  current_mode = "standby",
  autoware_ok = true,
  is_switchable = true,
  last_processed_id = 101
}
```

---

## 6. 用語使用時の注意

- **"request" の言及**: Switcher からの全体的な入力パッケージを指す
- **"command" の言及**: リクエスト内の具体的な操作指示を指す（単体での使用は避けるべき）
- **"status" の言及**: Core Logic から Switcher への応答を指す
- **"control" の言及**: Core Logic から Autoware への指示を指す

各コードやコメントでは、この定義に従って一貫性を保つこと。
