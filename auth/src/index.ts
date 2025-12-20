import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./utils/auth"; // Corrected path
import dotenv from "dotenv";

dotenv.config();

const app = express();
const PORT = Number(process.env.PORT) || 7860;
const Client = process.env.CLIENT_URL || "http://localhost:3000";

app.use(cors({
  origin: Client, // Adjust as needed
  credentials: true,
  methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
  allowedHeaders: ["Content-Type", "Authorization"],
}));

app.use(express.json());

// Better Auth Handler
app.use("/api/auth", toNodeHandler(auth));

app.get("/health", (req, res) => {
  res.status(200).json({ status: "ok" });
});

app.listen(PORT, "0.0.0.0", () => {
  console.log(`Auth server running on port ${PORT}`);
});
