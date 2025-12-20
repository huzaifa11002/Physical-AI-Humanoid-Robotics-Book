import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { db } from "../db";
import * as schema from "../db/schema";

export const auth = betterAuth({
    database: drizzleAdapter(db, {
        provider: "pg",
        schema: schema,
    }),
    emailAndPassword: {
        enabled: true,
    },
    trustedOrigins: ["http://localhost:3000", "https://physical-ai-humanoid-robotics-book-bice.vercel.app/"],
    advanced: {
        cookies: {
            sessionToken: {
                attributes: {
                    secure: true,
                    sameSite: "none",
                },
            },
        },
    },
    // Add other providers or plugins here if needed
});
