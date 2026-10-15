import * as React from "react";

import { ChatMessage, MemoryLocation, MemoryRecord } from "./types";
import "./agent-memory.css";

interface Props {
  connected: boolean;
  agentVisible: boolean;
  memoryVisible: boolean;
  agentEnabled: boolean;
  memoryEnabled: boolean;
  agentIdle: boolean | null;
  messages: ChatMessage[];
  records: MemoryRecord[];
  locations: MemoryLocation[];
  stats: Record<string, number>;
  onSendMessage: (text: string) => Promise<void>;
  onRefreshMemory: () => Promise<void>;
  onSearchMemory: (query: string) => Promise<MemoryRecord[]>;
}

function timeLabel(timestamp: number): string {
  const date = new Date(timestamp < 1e12 ? timestamp * 1000 : timestamp);
  return Number.isNaN(date.valueOf()) ? "时间未知" : date.toLocaleString();
}

function ChatEntry({ message }: { message: ChatMessage }): React.ReactElement {
  const role = message.role === "human" ? "Human" : message.role === "ai" ? "AI" : "Tool";
  return (
    <article className={`memory-chat-entry role-${message.role}`}>
      <header><strong>{role}{message.name ? ` · ${message.name}` : ""}</strong><time>{timeLabel(message.ts)}</time></header>
      {message.content && <div className="memory-message-text">{message.content}</div>}
      {message.tool_call_id && <small>Tool call: {message.tool_call_id}</small>}
      {message.tool_calls?.map((call) => (
        <details key={call.id}>
          <summary>调用工具：{call.name}</summary>
          <pre>{JSON.stringify(call.args ?? {}, null, 2)}</pre>
        </details>
      ))}
    </article>
  );
}

function MemoryCard({ record }: { record: MemoryRecord }): React.ReactElement {
  const safeThumbnail = record.thumbnail?.startsWith("data:image/jpeg;base64,") ? record.thumbnail : null;
  return (
    <article className="memory-card">
      {safeThumbnail && <img src={safeThumbnail} alt={record.label ? `记忆观察：${record.label}` : "记忆缩略图"} />}
      <strong>{record.label || `记忆 ${record.id}`}</strong>
      <time>{timeLabel(record.timestamp)}</time>
      <span>{record.coordinate_frame} · ({record.position.x.toFixed(2)}, {record.position.y.toFixed(2)}, {record.position.z.toFixed(2)})</span>
      {record.similarity !== null && <span>CLIP 分数 {record.similarity.toFixed(3)}</span>}
      <small>观察位置，不代表物体的 3D 位置</small>
    </article>
  );
}

export default function AgentMemoryPanel(props: Props): React.ReactElement {
  const { connected, agentVisible, memoryVisible, agentEnabled, memoryEnabled, agentIdle, messages, records, locations, stats,
    onSendMessage, onRefreshMemory, onSearchMemory } = props;
  const [tab, setTab] = React.useState<"chat" | "memory">(agentVisible ? "chat" : "memory");
  const [draft, setDraft] = React.useState("");
  const [query, setQuery] = React.useState("");
  const [busy, setBusy] = React.useState(false);
  const [memoryBusy, setMemoryBusy] = React.useState(false);
  const [error, setError] = React.useState("");
  const [memoryError, setMemoryError] = React.useState("");
  const memoryBusyRef = React.useRef(false);
  const chatListRef = React.useRef<HTMLDivElement>(null);
  const nearBottomRef = React.useRef(true);

  const refresh = React.useCallback(async () => {
    if (!connected || !memoryEnabled || memoryBusyRef.current) return;
    memoryBusyRef.current = true;
    setMemoryBusy(true);
    setMemoryError("");
    try { await onRefreshMemory(); }
    catch (err) { setMemoryError(err instanceof Error ? err.message : "记忆读取失败"); }
    finally { memoryBusyRef.current = false; setMemoryBusy(false); }
  }, [connected, memoryEnabled, onRefreshMemory]);

  React.useEffect(() => {
    if (tab !== "memory" || !connected || !memoryEnabled || query.trim()) return;
    void refresh();
    const interval = window.setInterval(() => void refresh(), 5000);
    return () => window.clearInterval(interval);
  }, [tab, connected, memoryEnabled, refresh, query]);

  React.useEffect(() => {
    if (nearBottomRef.current && chatListRef.current) {
      chatListRef.current.scrollTop = chatListRef.current.scrollHeight;
    }
  }, [messages.length]);

  React.useEffect(() => {
    if (tab === "chat" && !agentVisible && memoryVisible) setTab("memory");
    if (tab === "memory" && !memoryVisible && agentVisible) setTab("chat");
  }, [tab, agentVisible, memoryVisible]);

  const send = async (event: React.FormEvent) => {
    event.preventDefault();
    const text = draft.trim();
    if (!text || !connected || !agentEnabled || agentIdle !== true || busy) return;
    setBusy(true);
    setError("");
    try { await onSendMessage(text); setDraft(""); }
    catch (err) { setError(err instanceof Error ? err.message : "发送失败"); }
    finally { setBusy(false); }
  };

  const search = async (event: React.FormEvent) => {
    event.preventDefault();
    const text = query.trim();
    if (!text || !connected || !memoryEnabled || memoryBusyRef.current) return;
    memoryBusyRef.current = true;
    setMemoryBusy(true);
    setMemoryError("");
    try { await onSearchMemory(text); }
    catch (err) { setMemoryError(err instanceof Error ? err.message : "搜索失败"); }
    finally { memoryBusyRef.current = false; setMemoryBusy(false); }
  };

  const canSend = connected && agentEnabled && agentIdle === true && !busy;
  return (
    <section className="agent-memory-panel" aria-label="Agent 与空间记忆">
      <nav className="memory-tabs" aria-label="Agent 面板">
        {agentVisible && <button type="button" aria-selected={tab === "chat"} onClick={() => setTab("chat")}>对话</button>}
        {memoryVisible && <button type="button" aria-selected={tab === "memory"} onClick={() => setTab("memory")}>记忆</button>}
        <span className="agent-connection-status">{!connected ? "已断开" : !agentEnabled ? "Agent 不可用" : busy || agentIdle === false ? "Agent 忙碌" : "已连接"}</span>
      </nav>
      {!agentVisible && !memoryVisible && <p className="memory-empty" role="status">当前连接未启用 Agent 或空间记忆</p>}
      {tab === "chat" && agentVisible && <div className="agent-chat-panel">
        <div ref={chatListRef} className="memory-chat-list" aria-live="polite" onScroll={(event) => {
          const element = event.currentTarget;
          nearBottomRef.current = element.scrollHeight - element.scrollTop - element.clientHeight < 60;
        }}>
          {messages.length === 0 && <p className="memory-empty">暂无对话</p>}
          {messages.map((message) => <ChatEntry key={message.id} message={message} />)}
          {connected && agentIdle === false && <p className="memory-waiting" role="status">Agent 正在处理，请稍候…</p>}
        </div>
        <form className="agent-compose" onSubmit={send}>
          <label className="sr-only" htmlFor="agent-message">发送给 Agent 的消息</label>
          <textarea id="agent-message" value={draft} onChange={(event) => setDraft(event.target.value)} rows={2} maxLength={4000}
            placeholder="输入消息…" disabled={!canSend} />
          <button type="submit" disabled={!canSend || !draft.trim()}>发送</button>
          {!connected && <small role="status">连接后可发送</small>}
          {connected && agentIdle === null && <small role="status">等待 Agent 状态…</small>}
          {connected && agentIdle === false && <small role="status">Agent 忙碌中</small>}
          {error && <small className="memory-error" role="alert">{error}</small>}
        </form>
      </div>}
      {tab === "memory" && memoryVisible && <div className="agent-memory-content">
        <form className="memory-search" onSubmit={search}>
          <label htmlFor="memory-query">English text search (CLIP)</label>
          <input id="memory-query" value={query} onChange={(event) => setQuery(event.target.value)} placeholder="e.g. red chair" maxLength={512} disabled={!connected || !memoryEnabled} />
          <button type="submit" disabled={!connected || !memoryEnabled || !query.trim() || memoryBusy}>搜索 Top 3</button>
          <button type="button" onClick={() => void refresh()} disabled={!connected || !memoryEnabled || memoryBusy}>刷新</button>
        </form>
        <div className="memory-meta">记录 {stats.record_count ?? records.length} · 地点 {stats.location_count ?? locations.length} · 地图仅标记 odom 观察位置</div>
        {memoryError && <p className="memory-error" role="alert">{memoryError}</p>}
        {memoryBusy && <p className="memory-empty" role="status">正在读取记忆…</p>}
        {!memoryBusy && !memoryError && records.length === 0 && locations.length === 0 && <p className="memory-empty">暂无记忆记录或命名地点</p>}
        {!memoryBusy && <div className="memory-card-list">
          {records.map((record) => <MemoryCard key={record.id} record={record} />)}
          {locations.map((location) => <article className="memory-card memory-location" key={`location-${location.id}`}>
            <strong>地点 · {location.name}</strong><time>{timeLabel(location.timestamp)}</time>
            <span>{location.coordinate_frame} · ({location.position.x.toFixed(2)}, {location.position.y.toFixed(2)}, {location.position.z.toFixed(2)})</span>
            <small>命名观察位置，不代表物体的 3D 位置</small>
          </article>)}
        </div>}
      </div>}
    </section>
  );
}
