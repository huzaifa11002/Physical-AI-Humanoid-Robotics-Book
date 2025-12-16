import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
    baseURL: "https://huzaifa1102-better-auth.hf.space", // Base URL of your auth server
});
