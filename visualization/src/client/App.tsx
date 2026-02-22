import { useState } from "react";
import { trpc } from "./trpc";

function App() {
  const [name, setName] = useState("World");

  const greetingQuery = trpc.greeting.useQuery({ name });
  const itemsQuery = trpc.getItems.useQuery();

  return (
    <div style={{ padding: "2rem", fontFamily: "system-ui" }}>
      <h1>Vite + Express + tRPC</h1>
      <p style={{ color: "#666" }}>Simple fullstack setup with separate frontend and backend</p>

      <div style={{ marginTop: "2rem" }}>
        <h2>Greeting</h2>
        <input
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          placeholder="Enter your name"
          style={{ padding: "0.5rem", fontSize: "1rem" }}
        />
        <div style={{ marginTop: "1rem" }}>
          {greetingQuery.error && <p>Error: {greetingQuery.error.message}</p>}
          {greetingQuery.data && <p style={{ fontSize: "1.5rem" }}>{greetingQuery.data.message}</p>}
        </div>
      </div>

      <div style={{ marginTop: "2rem" }}>
        <h2>Items</h2>
        {itemsQuery.isLoading && <p>Loading items...</p>}
        {itemsQuery.error && <p>Error: {itemsQuery.error.message}</p>}
        {itemsQuery.data && (
          <ul>
            {itemsQuery.data.map((item) => (
              <li key={item.id}>{item.name}</li>
            ))}
          </ul>
        )}
      </div>
    </div>
  );
}

export default App;
